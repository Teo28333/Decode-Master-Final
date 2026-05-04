package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.constant.TurretConstants.*;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.ShooterEquation;

public class TurretSS {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final ServoImplEx     servo1;
    private final ServoImplEx     servo2;
    private final Telemetry       telemetry;
    private final ShooterEquation airTime = new ShooterEquation();

    // ── Outputs ───────────────────────────────────────────────────────────────
    private double servoPos = 0.5;

    // ── State ─────────────────────────────────────────────────────────────────
    private double  targetAngleRad       = 0.0; // absolute field-relative angle
    private double  filteredTargetAngleRad = 0.0;
    private double  actualTargetAngleRad = 0.0; // robot-relative angle (CCW+, front=±180°)
    private double  robotHeadingRad      = 0.0; // stored for telemetry
    private double  turretFieldX         = 0.0; // actual turret pivot position on field
    private double  turretFieldY         = 0.0;
    private double  filteredVelX         = 0.0; // low-pass filtered linear velocity
    private double  filteredVelY         = 0.0;
    private double  filteredHeadingVel   = 0.0; // low-pass filtered angular velocity
    private boolean robotNeedToTurn      = false;
    private boolean targetAngleInitialized = false;

    // ── Constructor ───────────────────────────────────────────────────────────
    public TurretSS(HardwareMap hwm, Telemetry telemetry,
                    String turretServo1, String turretServo2) {
        this.telemetry = telemetry;

        servo1 = hwm.get(ServoImplEx.class, turretServo1);
        servo2 = hwm.get(ServoImplEx.class, turretServo2);

        PwmControl.PwmRange fullRange = new PwmControl.PwmRange(500, 2500);
        servo1.setPwmRange(fullRange);
        servo2.setPwmRange(fullRange);
    }

    // ── Loop methods ──────────────────────────────────────────────────────────
    public void read() {
        // No sensors on this subsystem
    }

    public void write() {
        servo1.setPosition(servoPos);
        servo2.setPosition(servoPos);
    }

    public void update() {
        read();
        write();
        telemetry();
    }

    public void telemetry() {
        telemetry.addData("Turret target (deg, field)", Math.toDegrees(targetAngleRad));
        telemetry.addData("Turret target (deg, robot)", Math.toDegrees(actualTargetAngleRad));
        telemetry.addData("Robot heading (deg)",        Math.toDegrees(robotHeadingRad));
        telemetry.addData("Heading vel (deg/s)",        Math.toDegrees(filteredHeadingVel));
        telemetry.addData("Turret field pos",           "(" + turretFieldX + ", " + turretFieldY + ")");
        telemetry.addData("Filtered vel",               "(" + filteredVelX + ", " + filteredVelY + ")");
        telemetry.addData("Turret servo pos",           servoPos);
        telemetry.addData("Robot needs to turn",        robotNeedToTurn);
    }

    // ── Commands ──────────────────────────────────────────────────────────────

    /**
     * @param robotX           robot X position (inches)
     * @param robotY           robot Y position (inches)
     * @param robotHeading     robot heading (radians, CCW+)
     * @param robotVel         robot linear velocity vector (inches/s)
     * @param robotHeadingVel  robot angular velocity (radians/s, CCW+) — from Pedro
     * @param goalX            goal X position (inches)
     * @param goalY            goal Y position (inches)
     */
    public void aimAtTargetCMD(double robotX,         double robotY,
                               double robotHeading,   Vector robotVel,
                               double robotHeadingVel,
                               double goalX,          double goalY) {
        robotHeadingRad = robotHeading;
        if (robotVel == null) {
            robotVel = new Vector(0, 0);
        }

        // ── Angular velocity filter ───────────────────────────────────────────
        filteredHeadingVel = robotHeadingVel;

        // ── Linear velocity filter ────────────────────────────────────────────
        filteredVelX = robotVel.getXComponent();
        filteredVelY = robotVel.getYComponent();

        // ── Turret pivot position on field ────────────────────────────────────
        turretFieldX = robotX + TURRET_OFFSET_X * Math.cos(robotHeading)
                - TURRET_OFFSET_Y * Math.sin(robotHeading);
        turretFieldY = robotY + TURRET_OFFSET_X * Math.sin(robotHeading)
                + TURRET_OFFSET_Y * Math.cos(robotHeading);

        // ── Shoot-on-the-move goal compensation (linear velocity) ─────────────
        double distance = Math.hypot(goalX - turretFieldX, goalY - turretFieldY);
        double airtime  = airTime.getAirTime(distance);
        double adjGoalX = goalX - filteredVelX * airtime;
        double adjGoalY = goalY - filteredVelY * airtime;

        // ── Field-relative angle from turret pivot to goal ────────────────────
        targetAngleRad = Math.atan2(adjGoalY - turretFieldY, adjGoalX - turretFieldX);
        filteredTargetAngleRad = targetAngleRad;

        // ── Convert to robot-relative with heading prediction ─────────────────
        // Lead the heading by lookahead × angular velocity to compensate
        // for the turret servo being slower than the robot rotation speed
        double predictedHeading = robotHeading
                + filteredHeadingVel * HEADING_VEL_LOOKAHEAD_SEC;

        actualTargetAngleRad = normalizeAngle(
                filteredTargetAngleRad - predictedHeading + Math.PI
        );

        // ── Servo output (CW+, so negate CCW+ math angle) ─────────────────────
        double targetDeg = -Math.toDegrees(actualTargetAngleRad);
        double targetServoPos = angleDegToServoPos(targetDeg);
        if (Math.abs(targetServoPos - servoPos) > SERVO_DEADBAND) {
            servoPos = targetServoPos;
        }
        robotNeedToTurn  = targetDeg > maxSafeAngleDeg || targetDeg < minSafeAngleDeg;
    }

    public void failsafeParkCMD(double parkedServoPos) {
        servoPos = Range.clip(parkedServoPos, 0.0, 1.0);
        actualTargetAngleRad = 0.0;
        robotNeedToTurn = false;
        targetAngleInitialized = false;
        filteredVelX = 0.0;
        filteredVelY = 0.0;
        filteredHeadingVel = 0.0;
    }

    // ── Getters ───────────────────────────────────────────────────────────────
    public boolean robotNeedToTurn() {
        return robotNeedToTurn;
    }

    /** Robot-relative angle to goal in degrees (CCW+). Used by Robot to determine turn direction. */
    public double getTurretAngleDeg() {
        return Math.toDegrees(actualTargetAngleRad);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────
    public double getServoPos() {
        return servoPos;
    }

    private double angleDegToServoPos(double angleDeg) {
        double pos = (angleDeg - minAngleDeg) / (maxAngleDeg - minAngleDeg);
        return Range.clip(pos, 0.0, 1.0);
    }

    private double smoothAngle(double current, double target) {
        if (!targetAngleInitialized) {
            targetAngleInitialized = true;
            return target;
        }

        double delta = normalizeAngle(target - current);
        return normalizeAngle(current + (1.0 - TARGET_ANGLE_FILTER_ALPHA) * delta);
    }

    private static double normalizeAngle(double rad) {
        while (rad >  Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }
}
