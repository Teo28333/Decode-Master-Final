package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.constant.PtoConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.PIDFController;

public class PtoSS implements SubsystemInterface {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final Servo          servo;
    private final DcMotorEx      frontLeft;
    private final DcMotorEx      frontRight;
    private final DcMotorEx      backLeft;
    private final DcMotorEx      backRight;
    private final Telemetry      telemetry;
    private final PIDFController pidfController;

    // ── Outputs ───────────────────────────────────────────────────────────────
    private double servoPos = 0.0;
    private double motorPow = 0.0;

    // ── Sensor readings ───────────────────────────────────────────────────────
    private double liftCurrentPos = 0.0;

    // ── State ─────────────────────────────────────────────────────────────────
    private boolean canLift        = false;
    private boolean motorsAreReset = false; // never cleared — lift is one-time
    private boolean liftActive     = false;

    // ── Constructor ───────────────────────────────────────────────────────────
    public PtoSS(HardwareMap hwm, Telemetry telemetry,
                 String ptoServo,
                 String frontLeftMotor, String frontRightMotor,
                 String backLeftMotor,  String backRightMotor) {
        this.telemetry = telemetry;

        servo = hwm.get(Servo.class, ptoServo);

        frontLeft  = hwm.get(DcMotorEx.class, frontLeftMotor);
        frontRight = hwm.get(DcMotorEx.class, frontRightMotor);
        backLeft   = hwm.get(DcMotorEx.class, backLeftMotor);
        backRight  = hwm.get(DcMotorEx.class, backRightMotor);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        pidfController = new PIDFController(hwm);
    }

    // ── Loop methods ──────────────────────────────────────────────────────────
    @Override
    public void read() {
        liftCurrentPos = backLeft.getCurrentPosition();
    }

    @Override
    public void write() {
        servo.setPosition(servoPos);
        if (liftActive) {
            frontLeft.setPower(-motorPow);
            frontRight.setPower(-motorPow);
            backLeft.setPower(motorPow);
            backRight.setPower(motorPow);
        }
    }

    @Override
    public void update() {
        read();
        write();
        telemetry();
    }

    @Override
    public void telemetry() {
        telemetry.addData("PTO lift pos",    liftCurrentPos);
        telemetry.addData("PTO motor power", motorPow);
        telemetry.addData("PTO can lift",    canLift);
        telemetry.addData("PTO lift active", liftActive);
        telemetry.addData("PTO servo pos",   servoPos);
    }

    // ── Commands ──────────────────────────────────────────────────────────────
    public void engagePtoCMD() {
        servoPos = engagedPos;
        canLift  = true;
    }

    public void disengagePtoCMD() {
        servoPos   = disengagedPos;
        canLift    = false;
        liftActive = false;
        motorPow   = 0.0;
        // NOTE: motorsAreReset intentionally NOT cleared — lift is one-time only
    }

    public void activateLiftCMD() {
        if (!canLift) return;

        liftActive = true;

        // Reset encoders once at the very start of the first lift cycle
        if (!motorsAreReset) {
            motorReset();
            motorsAreReset = true;
        }

        motorPow = pidfController.calculate(ticksForLift, liftCurrentPos);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────
    private void motorReset() {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }
}
