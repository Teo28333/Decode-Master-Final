package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.constant.IntakeConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.constant.LedColors.*;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSS implements SubsystemInterface {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final DcMotorEx   intake1;
    private final DcMotorEx   intake2;
    private final Servo       gate;
    private final ServoImplEx led;
    private final Telemetry   telemetry;

    // ── Timers ────────────────────────────────────────────────────────────────
    private final ElapsedTime motor1Timer  = new ElapsedTime();
    private final ElapsedTime motor2Timer  = new ElapsedTime();
    private final ElapsedTime gateFailsafe = new ElapsedTime();

    // ── Sensor readings ───────────────────────────────────────────────────────
    private double motor1Current = 0.0;
    private double motor2Current = 0.0;

    // ── Outputs ───────────────────────────────────────────────────────────────
    private double firstMotorPow  = 0.0;
    private double secondMotorPow = 0.0;
    private double gatePos        = 0.0;
    private double ledColor       = INTAKE_IDLE;

    // ── State ─────────────────────────────────────────────────────────────────
    // motor2 detects the first ball (outer motor)
    // motor1 detects the second ball (inner motor) — can only trigger AFTER motor2
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
    @Override
    public void read() {
        motor1Current = intake1.getCurrent(CurrentUnit.MILLIAMPS);
        motor2Current = intake2.getCurrent(CurrentUnit.MILLIAMPS);
    }

    @Override
    public void write() {
        intake1.setPower(firstMotorPow);
        intake2.setPower(secondMotorPow);
        gate.setPosition(gatePos);
        led.setPosition(ledColor);
    }

    @Override
    public void telemetry() {
        telemetry.addData("Motor 1 current (mA)", motor1Current);
        telemetry.addData("Motor 2 current (mA)", motor2Current);
        telemetry.addData("Motor 1 stopped",      motor1Stopped);
        telemetry.addData("Motor 2 stopped",      motor2Stopped);
        telemetry.addData("Ball loaded",          isBallLoaded());
        telemetry.addData("Fully loaded",         isFullyLoaded());
        telemetry.addData("Need to turn",         needToTurn);
    }

    @Override
    public void update() {
        read();
        write();
        telemetry();
    }

    // ── Commands ──────────────────────────────────────────────────────────────
    public void intakeCMD() {
        // Close gate after failsafe delay following a transfer
        if (gateFailsafe.milliseconds() > gateTime) {
            gatePos = closeGatePos;
        }

        // Motor 2 ball detection (first ball — outer motor)
        if (motor2Current > firstCurrentThreshold) {
            if (motor2Timer.milliseconds() > motor2StopThreshold) motor2Stopped = true;
        } else {
            motor2Timer.reset();
        }

        // Motor 1 ball detection (second ball — inner motor)
        // IMPORTANT: motor1 can ONLY stop after motor2 has already stopped.
        // If motor2 has not stopped yet, motor1 timer is held at zero.
        if (motor2Stopped && motor1Current > secondCurrentThreshold) {
            if (motor1Timer.milliseconds() > motor1StopThreshold) motor1Stopped = true;
        } else if (!motor2Stopped) {
            motor1Timer.reset(); // prevent premature motor1 stop
        }

        // Power output
        secondMotorPow = motor2Stopped ? 0.0 : intakeSpeed;
        firstMotorPow  = motor1Stopped ? 0.0 : intakeSpeed;

        // LED feedback
        if      (motor1Stopped) ledColor = INTAKE_BALL_MOTOR1;
        else if (motor2Stopped) ledColor = INTAKE_BALL_MOTOR2;
        else                    ledColor = INTAKE_IDLE;
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

    /** Stop all motors and return LED to idle. Does NOT change gate position. */
    public void stop() {
        firstMotorPow  = 0.0;
        secondMotorPow = 0.0;
        ledColor       = INTAKE_IDLE;
        needToTurn     = false;
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
    public boolean needToTurn()    { return needToTurn;                       }
    public boolean isBallLoaded()  { return motor2Stopped;                    }
    public boolean isFullyLoaded() { return motor1Stopped && motor2Stopped;   }
}
