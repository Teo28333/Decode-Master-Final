package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.constant.ShooterConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.constant.LedColors.*;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.PIDFSController;
import org.firstinspires.ftc.teamcode.math.ShooterEquation;

public class ShooterSS implements SubsystemInterface {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final DcMotorEx       motor1;
    private final DcMotorEx       motor2;
    private final Servo           hood;
    private final ServoImplEx     led;
    private final Telemetry       telemetry;
    private final ShooterEquation shooterEquation = new ShooterEquation();
    private final PIDFSController pidfsController;

    // ── Outputs ───────────────────────────────────────────────────────────────
    private double motorPow = 0.0;
    private double hoodPos  = 0.0;
    private double ledColor = SHOOTER_SPINNING_UP;

    // ── Sensor readings ───────────────────────────────────────────────────────
    private double currentVelRPM   = 0.0;
    private double targetRPM       = 0.0;
    private double currentDistance = 0.0;

    // ── State ─────────────────────────────────────────────────────────────────
    private boolean tuningMode = false;

    // ── Constructor ───────────────────────────────────────────────────────────
    public ShooterSS(HardwareMap hwm, Telemetry telemetry,
                     String shooterMotor1, String shooterMotor2,
                     String hoodServo,    String led2) {
        this.telemetry = telemetry;

        motor1 = hwm.get(DcMotorEx.class, shooterMotor1);
        motor2 = hwm.get(DcMotorEx.class, shooterMotor2);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hood = hwm.get(Servo.class, hoodServo);

        led = hwm.get(ServoImplEx.class, led2);
        led.setPwmRange(new PwmControl.PwmRange(500, 2500));

        pidfsController = new PIDFSController(hwm);
    }

    // ── Loop methods ──────────────────────────────────────────────────────────
    @Override
    public void read() {
        currentVelRPM = tpsToRPM(motor1.getVelocity());
    }

    @Override
    public void write() {
        motor1.setPower(motorPow);
        motor2.setPower(motorPow);
        hood.setPosition(hoodPos);
        led.setPosition(ledColor);
    }

    @Override
    public void update() {
        read();
        write();
        telemetry();
    }

    @Override
    public void telemetry() {
        telemetry.addData("Shooter target (RPM)",  targetRPM);
        telemetry.addData("Shooter current (RPM)", currentVelRPM);
        telemetry.addData("Shooter error (RPM)",   targetRPM - currentVelRPM);
        telemetry.addData("Shooter power",         motorPow);
        telemetry.addData("Hood position",         hoodPos);
        telemetry.addData("Distance (in)",         currentDistance);
        telemetry.addData("At target",             isReady());
        telemetry.addData("Tuning mode",           tuningMode);
    }

    // ── Commands ──────────────────────────────────────────────────────────────
    public void shooterSpinCMD(double robotX, double robotY, double robotHeading,
                               Vector robotVel, double goalX, double goalY) {
        // Compensate goal position for robot velocity × ball air time
        double distance    = Math.hypot(goalX - robotX, goalY - robotY);
        double dt          = shooterEquation.getAirTime(distance);
        double adjGoalX    = goalX - robotVel.getXComponent() * dt;
        double adjGoalY    = goalY - robotVel.getYComponent() * dt;
        double adjDistance = Math.hypot(adjGoalX - robotX, adjGoalY - robotY);

        currentDistance = adjDistance;

        targetRPM = tuningMode ? tuningRPM     : shooterEquation.getTargetRPM(adjDistance);
        hoodPos   = tuningMode ? tuningHoodPos : shooterEquation.getHoodPos(adjDistance);
        motorPow  = pidfsController.calculate(targetRPM, currentVelRPM);
        ledColor  = isReady() ? SHOOTER_AT_TARGET : SHOOTER_SPINNING_UP;
    }

    public void setTuningMode(boolean enabled) {
        tuningMode = enabled;
    }

    // ── Getters ───────────────────────────────────────────────────────────────
    public boolean isReady() {
        return currentVelRPM > targetRPM - RPM_THRESHOLD
            && currentVelRPM < targetRPM + RPM_THRESHOLD / 3.0;
    }

    public double getCurrentRPM()     { return currentVelRPM;   }
    public double getTargetRPM()      { return targetRPM;        }
    public double getCurrentDistance(){ return currentDistance;  }

    // ── Helpers ───────────────────────────────────────────────────────────────
    private static double tpsToRPM(double tps) {
        return tps / 28.0 * 60.0;
    }
}
