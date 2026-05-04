package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.constant.ShooterConstants.*;

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

public class ShooterSS {

    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final Servo hood;
    private final ServoImplEx led;
    private final Telemetry telemetry;
    private final ShooterEquation shooterEquation = new ShooterEquation();
    private final PIDFSController pidfsController;

    private double motorPow = 0.0;
    private double hoodPos = 0.0;
    private double ledColor = 0.0;

    private double currentVelRPM = 0.0;
    private double targetRPM = 0.0;
    private double correctedTargetRPM = 0.0;
    private double rpmDrop = 0.0;
    private double hoodDrop = 0.0;
    private double currentDistance = 0.0;

    private boolean inShootingZone = false;
    private boolean tuningMode = false;

    private static final double ZONE_DIAGONAL = 10 * Math.sqrt(2);

    public ShooterSS(HardwareMap hwm, Telemetry telemetry,
                     String shooterMotor1, String shooterMotor2,
                     String hoodServo, String led2) {
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

    public void read() {
        currentVelRPM = Math.abs(tpsToRPM(motor1.getVelocity()));
    }

    public void write() {
        motor1.setPower(motorPow);
        motor2.setPower(motorPow);
        hood.setPosition(hoodPos);
        led.setPosition(ledColor);
    }

    public void update() {
        read();
        write();
        telemetry();
    }

    public void update(boolean showTelemetry) {
        read();
        write();
        if (showTelemetry) {
            telemetry();
        }
    }

    public void telemetry() {
        telemetry.addData("Shooter target (RPM)", targetRPM);
        telemetry.addData("Shooter corrected target (RPM)", correctedTargetRPM);
        telemetry.addData("Shooter current (RPM)", currentVelRPM);
        telemetry.addData("Shooter error (RPM)", targetRPM - currentVelRPM);
        telemetry.addData("Shooter RPM drop", rpmDrop);
        telemetry.addData("Shooter power", motorPow);
        telemetry.addData("Hood recovery drop", hoodDrop);
        telemetry.addData("Hood position", hoodPos);
        telemetry.addData("Distance (in)", currentDistance);
        telemetry.addData("At target", atTarget());
        telemetry.addData("In shooting zone", inShootingZone);
        telemetry.addData("Tuning mode", tuningMode);
    }

    public void shooterSpinCMD(double robotX, double robotY, double robotHeading,
                               Vector robotVel, double goalX, double goalY) {
        shooterSpinCMD(robotX, robotY, robotHeading, robotVel, goalX, goalY, false);
    }

    public void shooterSpinCMD(double robotX, double robotY, double robotHeading,
                               Vector robotVel, double goalX, double goalY,
                               boolean transferActive) {
        if (robotVel == null) {
            robotVel = new Vector(0, 0);
        }

        double distance = Math.hypot(goalX - robotX, goalY - robotY);
        double dt = shooterEquation.getAirTime(distance);
        double adjGoalX = goalX - robotVel.getXComponent() * dt;
        double adjGoalY = goalY - robotVel.getYComponent() * dt;
        double adjDistance = Math.hypot(adjGoalX - robotX, adjGoalY - robotY);

        currentDistance = adjDistance;
        inShootingZone = isInShootingZone(robotX, robotY);

        targetRPM = tuningMode ? tuningRPM : shooterEquation.getTargetRPM(adjDistance);
        double baseHoodPos = tuningMode ? tuningHoodPos : shooterEquation.getHoodPos(adjDistance);

        rpmDrop = Math.max(0.0, targetRPM - currentVelRPM);
        double activeRecoveryDrop = transferActive
                ? Math.max(0.0, rpmDrop - TRANSFER_RECOVERY_DEADBAND_RPM)
                : 0.0;
        correctedTargetRPM = targetRPM + activeRecoveryDrop * TRANSFER_RECOVERY_MULTIPLIER;

        double hoodDropRatio = Math.min(rpmDrop, MAX_HOOD_DROP_RPM_LOSS) / MAX_HOOD_DROP_RPM_LOSS;
        hoodDrop = transferActive ? MAX_HOOD_RECOVERY_DROP * hoodDropRatio : 0.0;
        hoodPos = Math.max(0.5, baseHoodPos - hoodDrop);

        motorPow = pidfsController.calculate(correctedTargetRPM, currentVelRPM);
        ledColor = inShootingZone ? 0.5 : 0.0;
    }

    public void setTuningMode(boolean enabled) {
        tuningMode = enabled;
    }

    public boolean isReady() {
        return atTarget();
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCorrectedTargetRPM() {
        return correctedTargetRPM;
    }

    public double getCurrentVelRPM() {
        return currentVelRPM;
    }

    public double getHoodPos() {
        return hoodPos;
    }

    private boolean atTarget() {
        return currentVelRPM > targetRPM - RPM_THRESHOLD
                && currentVelRPM < targetRPM + RPM_THRESHOLD / 3.0;
    }

    private boolean isInShootingZone(double x, double y) {
        return isInBackZone(x, y) || isInFrontZone(x, y);
    }

    private boolean isInBackZone(double x, double y) {
        return y <= x - 48 + ZONE_DIAGONAL && y <= -x + 96 + ZONE_DIAGONAL;
    }

    private boolean isInFrontZone(double x, double y) {
        return y >= -x + 144 - ZONE_DIAGONAL && y >= x - ZONE_DIAGONAL;
    }

    private static double tpsToRPM(double tps) {
        return tps / 28.0 * 60.0;
    }
}
