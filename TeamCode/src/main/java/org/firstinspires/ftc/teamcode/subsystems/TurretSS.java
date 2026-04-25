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
    private final ServoImplEx    servo1;
    private final ServoImplEx    servo2;
    private final Telemetry      telemetry;
    private final ShooterEquation airTime = new ShooterEquation();

    // ── Outputs ───────────────────────────────────────────────────────────────
    private double servoPos = 0.5;

    // ── State ─────────────────────────────────────────────────────────────────
    private double  targetAngleRad       = 0.0; // absolute field-relative angle
    private double  actualTargetAngleRad = 0.0; // robot-relative angle
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
        telemetry.addData("Turret servo pos",           servoPos);
        telemetry.addData("Robot needs to turn",        robotNeedToTurn);
    }

    // ── Commands ──────────────────────────────────────────────────────────────
    public void aimAtTargetCMD(double robotX, double robotY, double robotHeading,
                            Vector robotVel, double goalX, double goalY) {
        // Compensate goal position for robot velocity × ball air time
        double distance = Math.hypot(goalX - robotX, goalY - robotY);
        double dt       = airTime.getAirTime(distance);
        double adjGoalX = goalX - robotVel.getXComponent() * dt;
        double adjGoalY = goalY - robotVel.getYComponent() * dt;

        targetAngleRad       = Math.atan2(adjGoalY - robotY, adjGoalX - robotX);
        actualTargetAngleRad = targetAngleRad + robotHeading;

        double targetDeg = Math.toDegrees(actualTargetAngleRad);
        servoPos         = angleDegToServoPos(targetDeg);
        robotNeedToTurn  = Math.abs(targetDeg) > maxSafeAngleDeg;
    }

    // ── Getters ───────────────────────────────────────────────────────────────
    public boolean robotNeedToTurn() {
        return robotNeedToTurn;
    }

    // ── Helpers ───────────────────────────────────────────────────────────────
    private double angleDegToServoPos(double angleDeg) {
        double pos = (angleDeg - minAngleDeg) / (maxAngleDeg - minAngleDeg);
        return Range.clip(pos, 0.0, 1.0);
    }
}
