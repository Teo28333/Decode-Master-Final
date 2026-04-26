package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.constant.TurretConstants.*;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.ShooterEquation;

public class TurretSS implements SubsystemInterface {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final ServoImplEx     servo1;
    private final ServoImplEx     servo2;
    private final Telemetry       telemetry;
    private final ShooterEquation airTime = new ShooterEquation();

    // ── Outputs ───────────────────────────────────────────────────────────────
    private double servoPos = 0.5;

    // ── State ─────────────────────────────────────────────────────────────────
    private double  targetAngleRad       = 0.0; // absolute field-relative angle
    private double  actualTargetAngleRad = 0.0; // robot-relative angle (CCW+, front=±180°)
    private double  robotHeadingRad      = 0.0; // stored for telemetry
    private double  turretFieldX         = 0.0; // actual turret pivot on field
    private double  turretFieldY         = 0.0;
    private double  filteredVelX         = 0.0; // low-pass filtered linear velocity
    private double  filteredVelY         = 0.0;
    private double  filteredHeadingVel   = 0.0; // low-pass filtered angular velocity
    private boolean robotNeedToTurn      = false;

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
    @Override
    public void read() {
        // No sensors on this subsystem
    }

    @Override
    public void write() {
        servo1.setPosition(servoPos);
        servo2.setPosition(servoPos);
    }

    @Override
    public void update() {
        read();
        write();
        telemetry();
    }

    @Override
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
     * Aim the turret at the goal, compensating for:
     *   - Robot linear velocity (shoot-on-the-move)
     *   - Robot angular velocity (turret servo lag during rotation)
     *   - Turret physical offset from robot center
     *   -
     * Coordinate conventions:
     *   - Field: CCW+, 0°=+X, 90°=+Y (matches PedroPathing and atan2)
     *   - Turret zero: back of robot → add π to convert
     *   - Servo: CW+ → negate math angle before converting
     *
     * @param robotX           robot X position (inches)
     * @param robotY           robot Y position (inches)
     * @param robotHeading     robot heading (radians, CCW+, from PedroPathing)
     * @param robotVel         robot linear velocity vector (inches/s)
     * @param robotHeadingVel  robot angular velocity (rad/s, CCW+, from Pedro)
     * @param goalX            goal X position (inches)
     * @param goalY            goal Y position (inches)
     */
    public void aimAtTargetCMD(double robotX,         double robotY,
                               double robotHeading,   Vector robotVel,
                               double robotHeadingVel,
                               double goalX,          double goalY) {
        robotHeadingRad = robotHeading;

        // ── Angular velocity filter ───────────────────────────────────────────
        filteredHeadingVel = HEADING_VEL_FILTER_ALPHA * filteredHeadingVel
                           + (1.0 - HEADING_VEL_FILTER_ALPHA) * robotHeadingVel;

        // ── Linear velocity filter ────────────────────────────────────────────
        filteredVelX = VELOCITY_FILTER_ALPHA * filteredVelX
                     + (1.0 - VELOCITY_FILTER_ALPHA) * robotVel.getXComponent();
        filteredVelY = VELOCITY_FILTER_ALPHA * filteredVelY
                     + (1.0 - VELOCITY_FILTER_ALPHA) * robotVel.getYComponent();

        // ── Turret pivot position on field ────────────────────────────────────
        // Rotate local offset into field frame using robot heading
        turretFieldX = robotX + TURRET_OFFSET_X * Math.cos(robotHeading)
                              - TURRET_OFFSET_Y * Math.sin(robotHeading);
        turretFieldY = robotY + TURRET_OFFSET_X * Math.sin(robotHeading)
                              + TURRET_OFFSET_Y * Math.cos(robotHeading);

        // ── Shoot-on-the-move compensation ───────────────────────────────────
        double distance = Math.hypot(goalX - turretFieldX, goalY - turretFieldY);
        double airtime  = airTime.getAirTime(distance);
        double adjGoalX = goalX - filteredVelX * airtime;
        double adjGoalY = goalY - filteredVelY * airtime;

        // ── Field-relative angle from turret pivot to goal ────────────────────
        targetAngleRad = Math.atan2(adjGoalY - turretFieldY, adjGoalX - turretFieldX);

        // ── Convert to robot-relative ─────────────────────────────────────────
        // - subtract heading  (Pedro CCW+ matches atan2 → normal subtraction)
        // - add π             (turret zero is at robot back)
        // - predict heading   (lead by lookahead × angular velocity for servo lag)
        double predictedHeading = robotHeading
                                + filteredHeadingVel * HEADING_VEL_LOOKAHEAD_SEC;

        actualTargetAngleRad = normalizeAngle(
                targetAngleRad - predictedHeading + Math.PI
        );

        // ── Servo output ──────────────────────────────────────────────────────
        // Negate because servo is CW+ but math angles are CCW+
        double targetDeg = -Math.toDegrees(actualTargetAngleRad);
        servoPos         = angleDegToServoPos(targetDeg);
        robotNeedToTurn  = Math.abs(targetDeg) > maxSafeAngleDeg;
    }

    // ── Getters ───────────────────────────────────────────────────────────────
    public boolean robotNeedToTurn() {
        return robotNeedToTurn;
    }

    /**
     * Robot-relative angle to goal in degrees (CCW+).
     * Used by Robot to determine auto-rotate direction during transfer.
     */
    public double getTurretAngleDeg() {
        return Math.toDegrees(actualTargetAngleRad);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    /**
     * Convert angle in degrees to servo position [0, 1].
     * Uses general form (angleDeg - minAngleDeg) / (maxAngleDeg - minAngleDeg)
     * which works correctly for both symmetric and asymmetric ranges.
     */
    private double angleDegToServoPos(double angleDeg) {
        double pos = (angleDeg - minAngleDeg) / (maxAngleDeg - minAngleDeg);
        return Range.clip(pos, 0.0, 1.0);
    }

    /** Normalize angle to [-π, π] to prevent wrap-around errors. */
    private static double normalizeAngle(double rad) {
        while (rad >  Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }
}
