package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.constant.IntakeConstants.*;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSS {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final DcMotorEx   intake1;
    private final DcMotorEx   intake2;
    private final Servo       gate;
    private final ServoImplEx led;
    private final Telemetry   telemetry;

    // ── Timers ────────────────────────────────────────────────────────────────
    private final ElapsedTime motor1Timer   = new ElapsedTime();
    private final ElapsedTime motor2Timer   = new ElapsedTime();
    private final ElapsedTime gateFailsafe  = new ElapsedTime();

    // ── Sensor readings ───────────────────────────────────────────────────────
    private double motor1Current = 0.0;
    private double motor2Current = 0.0;

    // ── Outputs ───────────────────────────────────────────────────────────────
    private double firstMotorPow  = 0.0;
    private double secondMotorPow = 0.0;
    private double gatePos        = 0.0;
    private double ledColor       = 0.0;

    // ── State ─────────────────────────────────────────────────────────────────
    private boolean motor1Stopped = false;
    private boolean motor2Stopped = false;
    private boolean needToTurn    = false;

    // ── Constants ─────────────────────────────────────────────────────────────
    private static final double ZONE_DIAGONAL = 10 * Math.sqrt(2);

    // ── Constructor ───────────────────────────────────────────────────────────
    public IntakeSS(HardwareMap hwm, Telemetry telemetry,
                    String intakeMotor1, String intakeMotor2,
                    String gateServo,   String led1) {
        this.telemetry = telemetry;

        intake1 = hwm.get(DcMotorEx.class, intakeMotor1);
        intake2 = hwm.get(DcMotorEx.class, intakeMotor2);
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gate = hwm.get(Servo.class, gateServo);

        led = hwm.get(ServoImplEx.class, led1);
        led.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    // ── Loop methods ──────────────────────────────────────────────────────────
    public void read() {
        motor1Current = intake1.getCurrent(CurrentUnit.MILLIAMPS);
        motor2Current = intake2.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public void write() {
        intake1.setPower(firstMotorPow);
        intake2.setPower(secondMotorPow);
        gate.setPosition(gatePos);
        led.setPosition(ledColor);
    }

    public void telemetry() {
        telemetry.addData("Motor 1 current (mA)", motor1Current);
        telemetry.addData("Motor 2 current (mA)", motor2Current);
        telemetry.addData("Motor 1 stopped",      motor1Stopped);
        telemetry.addData("Motor 2 stopped",      motor2Stopped);
    }

    public void update() {
        read();
        write();
        telemetry();
    }

    // ── Commands ──────────────────────────────────────────────────────────────

    // TODO: Add the command here

    public void intakeCMD() {
        // Close gate after failsafe delay following a transfer
        if (gateFailsafe.milliseconds() > gateTime) {
            gatePos = closeGatePos;
        }

        // Motor 2 stall detection
        if (motor2Current > firstCurrentThreshold) {
            if (motor2Timer.milliseconds() > motor2StopThreshold) motor2Stopped = true;
        } else {
            motor2Timer.reset();
        }

        // Motor 1 stall detection
        if (motor1Current > secondCurrentThreshold) {
            if (motor1Timer.milliseconds() > motor1StopThreshold) motor1Stopped = true;
        } else {
            motor1Timer.reset();
        }

        // Power output
        secondMotorPow = motor2Stopped ? 0.0 : intakeSpeed;
        firstMotorPow  = motor1Stopped ? 0.0 : intakeSpeed;

        // LED feedback
        if      (motor1Stopped) ledColor = 0.50;
        else if (motor2Stopped) ledColor = 0.65;
        else                    ledColor = 0.28;
    }

    public void outtakeCMD() {
        gatePos = openGatePos;

        resetIntake();
        firstMotorPow  = outtakeSpeed;
        secondMotorPow = outtakeSpeed;
    }

    public void transferCMD(double x, double y, boolean ready, boolean aimedAtTarget) {
        needToTurn = ready && !aimedAtTarget;

        if (!ready || !aimedAtTarget || !isInShootingZone(x, y)) {
            firstMotorPow  = 0.0;
            secondMotorPow = 0.0;
            if (gateFailsafe.milliseconds() > gateTime) {
                gatePos = closeGatePos;
            }
            return;
        }

        // Only reset detection state once when the transfer begins
        if (gatePos != openGatePos) {
            resetIntake();
            gateFailsafe.reset();
        }

        double speed = isInBackZone(x, y) ? farTransferSpeed : closeTransferSpeed;
        gatePos        = openGatePos;
        firstMotorPow  = speed;
        secondMotorPow = speed;
    }

    public void resetIntake() {
        motor1Stopped = false;
        motor2Stopped = false;
        motor1Timer.reset();
        motor2Timer.reset();
    }

    // ── Zone helpers ──────────────────────────────────────────────────────────
    private boolean isInShootingZone(double x, double y) {
        return isInBackZone(x, y) || isInFrontZone(x, y);
    }

    private boolean isInBackZone(double x, double y) {
        return y <= x - 48 + ZONE_DIAGONAL && y <= -x + 96 + ZONE_DIAGONAL;
    }

    private boolean isInFrontZone(double x, double y) {
        return y >= -x + 144 - ZONE_DIAGONAL && y >= x - ZONE_DIAGONAL;
    }

    // ── Getters ───────────────────────────────────────────────────────────────
    public boolean needToTurn() {
        return needToTurn;
    }
}