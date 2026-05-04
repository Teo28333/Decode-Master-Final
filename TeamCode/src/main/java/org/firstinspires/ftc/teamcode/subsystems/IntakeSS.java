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
    private final ElapsedTime motor1Timer  = new ElapsedTime();
    private final ElapsedTime motor2Timer  = new ElapsedTime();
    private final ElapsedTime gateFailsafe = new ElapsedTime();
    private final ElapsedTime transferJamTimer = new ElapsedTime();

    // ── Sensor readings ───────────────────────────────────────────────────────
    private double motor1Current = 0.0;
    private double motor2Current = 0.0;

    // ── Outputs ───────────────────────────────────────────────────────────────
    private double firstMotorPow  = 0.0;
    private double secondMotorPow = 0.0;
    private double gatePos        = closeGatePos;
    private double ledColor       = 0.3;

    // ── State ─────────────────────────────────────────────────────────────────
    private boolean motor1Stopped = false;
    private boolean motor2Stopped = false;
    private boolean needToTurn    = false;
    private boolean isIntaking = false;
    private boolean isTransferring = false;
    private boolean gateClosingForIntake = false;
    private boolean gateOpeningForTransfer = false;
    private boolean ignoreShootingZoneCheck = false;
    private boolean ignoreAimCheck = false;
    private BallState ballState = BallState.EMPTY;
    private TransferState transferState = TransferState.IDLE;

    public enum BallState {
        EMPTY,
        ONE,
        FULL
    }

    public enum TransferState {
        IDLE,
        WAITING_GATE,
        WAITING_READY,
        FEEDING,
        JAMMED
    }

    // ── Constants ─────────────────────────────────────────────────────────────
    private static final double ZONE_DIAGONAL    = 10 * Math.sqrt(2);
    private static final double LED_IDLE         = 0.8;

    // ── Constructor ───────────────────────────────────────────────────────────
    public IntakeSS(HardwareMap hwm, Telemetry telemetry,
                    String intakeMotor1, String intakeMotor2,
                    String gateServo,   String led1) {
        this.telemetry = telemetry;

        intake1 = hwm.get(DcMotorEx.class, intakeMotor1);
        intake2 = hwm.get(DcMotorEx.class, intakeMotor2);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gate = hwm.get(Servo.class, gateServo);
        gatePos = closeGatePos;
        gate.setPosition(gatePos);

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
        telemetry.addData("Intake ball state",    ballState);
        telemetry.addData("Transfer state",       transferState);
        telemetry.addData("Gate position",        gatePos);
    }

    public void update() {
        read();
        write();
        telemetry();
    }

    // ── Commands ──────────────────────────────────────────────────────────────

    public void intakeCMD() {
        if (!isIntaking) {
            gateClosingForIntake = false;
        }
        isIntaking = true;
        isTransferring = false;
        if (!gateClosingForIntake || gatePos != closeGatePos) {
            gateFailsafe.reset();
            gateClosingForIntake = true;
        }
        gateOpeningForTransfer = false;
        transferState = TransferState.IDLE;
        gatePos = closeGatePos;

        if (gateFailsafe.milliseconds() < gateSettleTime) {
            firstMotorPow  = 0.0;
            secondMotorPow = 0.0;
            ledColor       = LED_IDLE;
            return;
        }

        // Motor 2 ball detection
        if (motor2Current > firstCurrentThreshold) {
            if (motor2Timer.milliseconds() > motor2StopThreshold) motor2Stopped = true;
        } else {
            motor2Timer.reset();
        }

        // Motor 1 ball detection
        if (motor1Current > secondCurrentThreshold && motor2Stopped) {
            if (motor1Timer.milliseconds() > motor1StopThreshold) motor1Stopped = true;
        } else {
            motor1Timer.reset();
        }

        updateBallState();

        // Power output
        secondMotorPow = motor2Stopped ? 0.0 : secondIntakeSpeed;
        firstMotorPow  = motor1Stopped ? 0.0 : intakeSpeed;

        // LED feedback
        if      (motor1Stopped) ledColor = 0.50;
        else if (motor2Stopped) ledColor = 0.35;
        else                    ledColor = LED_IDLE;
    }

    public void outtakeCMD() {
        isIntaking = false;
        isTransferring = false;
        gateClosingForIntake = false;
        gateOpeningForTransfer = false;
        transferState = TransferState.IDLE;
        gatePos = openGatePos;

        resetIntake();
        ballState = BallState.EMPTY;
        firstMotorPow  = outtakeSpeed;
        secondMotorPow = outtakeSpeed;
    }

    public void transferCMD(double x, double y, boolean ready, boolean aimedAtTarget) {
        isIntaking = false;
        boolean canAim = aimedAtTarget || ignoreAimCheck;
        needToTurn = ready && !canAim;

        if (!isTransferring) {
            resetIntake();
            transferJamTimer.reset();
            transferState = TransferState.WAITING_GATE;
            isTransferring = true;
            gateClosingForIntake = false;
            gateOpeningForTransfer = false;
        }

        if (!gateOpeningForTransfer || gatePos != openGatePos) {
            gateFailsafe.reset();
            gateOpeningForTransfer = true;
        }
        gatePos = openGatePos;

        if (transferState == TransferState.JAMMED) {
            firstMotorPow = 0.0;
            secondMotorPow = 0.0;
            return;
        }

        if (!ready || !canAim || !isInShootingZone(x, y)
                || gateFailsafe.milliseconds() < gateSettleTime) {
            firstMotorPow  = 0.0;
            secondMotorPow = 0.0;
            transferJamTimer.reset();
            transferState = gateFailsafe.milliseconds() < gateSettleTime
                    ? TransferState.WAITING_GATE
                    : TransferState.WAITING_READY;
            return;
        }

        if (motor1Current > transferJamCurrentThreshold
                && motor2Current > transferJamCurrentThreshold) {
            if (transferJamTimer.milliseconds() > transferJamTime) {
                firstMotorPow = 0.0;
                secondMotorPow = 0.0;
                transferState = TransferState.JAMMED;
                return;
            }
        } else {
            transferJamTimer.reset();
        }

        double speed = isInBackZone(x, y) ? farTransferSpeed : closeTransferSpeed;
        firstMotorPow  = speed;
        secondMotorPow = speed;
        transferState = TransferState.FEEDING;
    }

    /** Stop all motors, close the gate, and return LED to idle. */
    public void stop() {
        firstMotorPow  = 0.0;
        secondMotorPow = 0.0;
        gatePos        = closeGatePos;
        ledColor       = LED_IDLE;
        isIntaking = false;
        isTransferring = false;
        gateClosingForIntake = false;
        gateOpeningForTransfer = false;
        needToTurn     = false;
        transferState  = TransferState.IDLE;
    }

    public void resetIntake() {
        motor1Stopped = false;
        motor2Stopped = false;
        ballState = BallState.EMPTY;
        gateClosingForIntake = false;
        gateOpeningForTransfer = false;
        transferJamTimer.reset();
        motor1Timer.reset();
        motor2Timer.reset();
    }

    private void updateBallState() {
        if (motor1Stopped) {
            ballState = BallState.FULL;
        } else if (motor2Stopped) {
            ballState = BallState.ONE;
        } else {
            ballState = BallState.EMPTY;
        }
    }

    // ── Zone helpers ──────────────────────────────────────────────────────────
    private boolean isInShootingZone(double x, double y) {
        return ignoreShootingZoneCheck || isInBackZone(x, y) || isInFrontZone(x, y);
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

    public BallState getBallState() {
        return ballState;
    }

    public TransferState getTransferState() {
        return transferState;
    }

    public boolean isFull() {
        return ballState == BallState.FULL;
    }

    public void setIgnoreShootingZoneCheck(boolean ignoreShootingZoneCheck) {
        this.ignoreShootingZoneCheck = ignoreShootingZoneCheck;
    }

    public void setIgnoreAimCheck(boolean ignoreAimCheck) {
        this.ignoreAimCheck = ignoreAimCheck;
    }
}
