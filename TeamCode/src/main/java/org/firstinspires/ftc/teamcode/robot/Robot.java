package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.LiftCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.PtoSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;

public class Robot {

    // ── Subsystems ────────────────────────────────────────────────────────────
    private final Follower  follower;
    private final IntakeSS  intake;
    private final ShooterSS shooter;
    private final TurretSS  turret;
    private final PtoSS     pto;

    // ── Commands ──────────────────────────────────────────────────────────────
    private final IntakeCommands intakeCommands;
    private final LiftCommands   liftCommands;

    // ── Alliance ──────────────────────────────────────────────────────────────
    private final boolean isBlueAlliance;

    private static final double HEADING_RED  = 0.0;
    private static final double HEADING_BLUE = 180;

    // ── Goal position ─────────────────────────────────────────────────────────
    // ── Auto-rotate tuning ────────────────────────────────────────────────────
    // How fast the robot rotates to face the goal during transfer (0.0 - 1.0)
    private static final double TURN_CORRECTION_POWER = 0.375;

    // ── Button edge detection ─────────────────────────────────────────────────
    private boolean lastIntake    = false;
    private boolean lastLift      = false;
    private boolean lastDisengage = false;
    private boolean lastPoseResetStart = false;
    private boolean lastPoseResetCenter = false;
    private boolean isInTuning;

    // ── Constructor ───────────────────────────────────────────────────────────
    public Robot(HardwareMap hwm, Telemetry telemetry, boolean isBlueAlliance, boolean tuning) {
        this.isBlueAlliance = isBlueAlliance;
        this.isInTuning     = tuning;

        follower = Constants.createFollower(hwm);

        intake  = new IntakeSS(hwm, telemetry, "intake1", "intake2", "gate", "led1");
        shooter = new ShooterSS(hwm, telemetry, "shooter1", "shooter2", "hood", "led2");
        turret  = new TurretSS(hwm, telemetry, "turret1", "turret2");
        pto     = new PtoSS(hwm, telemetry, "pto",
                "frontLeft", "frontRight", "backLeft", "backRight");

        intakeCommands = new IntakeCommands(intake);
        liftCommands   = new LiftCommands(pto);

        PanelsDebug.init();

        // Safety: force PTO disengaged at startup before any loop runs
        pto.disengagePtoCMD();
        pto.write();
    }

    // ── Robot start ───────────────────────────────────────────────────────────
    public void start() {
        if (!PoseStorage.hasValidPose()) {
            PoseStorage.setCurrentPose(PoseStorage.allianceStartPose(isBlueAlliance));
        }

        follower.setStartingPose(PoseStorage.currentPose);
        follower.setMaxPower(RobotConstants.DRIVE_SPEED_MULTIPLIER);
        follower.startTeleopDrive();
    }

    // ── Main teleop loop ──────────────────────────────────────────────────────
    public void update(Gamepad gamepad1) {

        shooter.setTuningMode(isInTuning);

        // ── Pose & velocity (needed before driving block) ─────────────────────
        Pose   pose     = follower.getPose();
        double robotX   = pose.getX();
        double robotY   = pose.getY();
        double heading  = pose.getHeading();
        Vector robotVel = follower.getVelocity();

        // ── Turret & shooter (before driving so robotNeedToTurn is fresh) ──────
        turret.aimAtTargetCMD(robotX, robotY, heading, robotVel, follower.getAngularVelocity(),
                aimGoalX(), aimGoalY());
        shooter.shooterSpinCMD(robotX, robotY, heading, robotVel,
                shootingGoalX(), shootingGoalY(), intakeCommands.isTransferring());

        // ── Driving ───────────────────────────────────────────────────────────
        double headingOffset = isBlueAlliance ? HEADING_BLUE : HEADING_RED;

        // During transfer, if the robot needs to turn to face the goal,
        // override the driver's rotation input with an auto-correction
        double turnInput = -gamepad1.right_stick_x * 0.75;
        if (intakeCommands.isTransferring() && turret.robotNeedToTurn()) {
            // Determine turn direction from the turret's robot-relative angle:
            // positive angle → goal is to the left → turn CCW (positive turn)
            // negative angle → goal is to the right → turn CW (negative turn)
            turnInput = Math.copySign(TURN_CORRECTION_POWER, turret.getTurretAngleDeg());
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * RobotConstants.DRIVE_SPEED_MULTIPLIER,
                -gamepad1.left_stick_x * RobotConstants.DRIVE_SPEED_MULTIPLIER,
                turnInput * RobotConstants.DRIVE_SPEED_MULTIPLIER,
                false,
                Math.toRadians(headingOffset)
        );
        follower.update();
        PoseStorage.setCurrentPose(follower.getPose());

        // ── Edge detection for toggle buttons ─────────────────────────────────
        boolean intakePressed    = gamepad1.right_bumper && !lastIntake;
        boolean liftPressed      = gamepad1.y            && !lastLift;
        boolean disengagePressed = gamepad1.left_bumper  && !lastDisengage;
        boolean resetCenterPressed = gamepad1.dpad_up && !lastPoseResetCenter;
        boolean resetStartPressed = gamepad1.dpad_down && !lastPoseResetStart;

        if (resetCenterPressed) {
            resetPose(PoseStorage.fieldCenterPose());
        } else if (resetStartPressed) {
            resetPose(PoseStorage.allianceStartPose(isBlueAlliance));
        }

        // ── Intake (toggle on right bumper) ───────────────────────────────────
        if (intakePressed) {
            if (intakeCommands.isIntaking()) intakeCommands.idle();
            else                             intakeCommands.intake();
        }

        // ── Outtake (active while B held) ─────────────────────────────────────
        if (gamepad1.b) {
            if (!intakeCommands.isOuttaking()) intakeCommands.outtake();
        } else if (intakeCommands.isOuttaking()) {
            intakeCommands.idle();
        }

        // ── Transfer (active while left bumper held) ───────────────────────────
        if (gamepad1.x) {
            if (!intakeCommands.isTransferring()) intakeCommands.transfer();
        } else if (intakeCommands.isTransferring()) {
            intakeCommands.idle();
        }

        // ── Lift commands ─────────────────────────────────────────────────────
        if (liftPressed)      liftCommands.lift();
        if (disengagePressed) liftCommands.disengage();

        // ── Save button states for next frame ─────────────────────────────────
        lastIntake    = gamepad1.right_bumper;
        lastLift      = gamepad1.y;
        lastDisengage = gamepad1.left_bumper;
        lastPoseResetCenter = gamepad1.dpad_up;
        lastPoseResetStart = gamepad1.dpad_down;

        // ── Command updates ───────────────────────────────────────────────────
        intakeCommands.update(
                robotX, robotY,
                shooter.isReady(),
                !turret.robotNeedToTurn()
        );
        liftCommands.update();

        // ── Subsystem updates ─────────────────────────────────────────────────
        intake.update();
        shooter.update();
        turret.update();
        pto.update();
        PanelsDebug.update(follower, shooter, turret, intakeCommands.isTransferring());
    }

    // ── Getters ───────────────────────────────────────────────────────────────
    public Follower getFollower()    { return follower;        }
    public boolean  isBlueAlliance() { return isBlueAlliance;  }

    private void resetPose(Pose pose) {
        if (PoseStorage.isValid(pose)) {
            follower.setPose(pose);
            PoseStorage.setCurrentPose(pose);
        }
    }

    private double aimGoalX() {
        return isBlueAlliance ? RobotConstants.AIM_GOAL_X_BLUE : RobotConstants.AIM_GOAL_X_RED;
    }

    private double aimGoalY() {
        return isBlueAlliance ? RobotConstants.AIM_GOAL_Y_BLUE : RobotConstants.AIM_GOAL_Y_RED;
    }

    private double shootingGoalX() {
        return isBlueAlliance ? RobotConstants.SHOOTING_GOAL_X_BLUE : RobotConstants.SHOOTING_GOAL_X_RED;
    }

    private double shootingGoalY() {
        return isBlueAlliance ? RobotConstants.SHOOTING_GOAL_Y_BLUE : RobotConstants.SHOOTING_GOAL_Y_RED;
    }
}
