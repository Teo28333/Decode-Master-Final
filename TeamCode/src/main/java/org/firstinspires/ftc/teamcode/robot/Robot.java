package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.math.ShooterEquation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.LimelightLocalizerSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;

import static org.firstinspires.ftc.teamcode.subsystems.constant.TurretConstants.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.subsystems.constant.TurretConstants.TURRET_OFFSET_Y;

public class Robot {

    // ── Subsystems ────────────────────────────────────────────────────────────
    private final Telemetry telemetry;
    private final Follower  follower;
    private final IntakeSS  intake;
    private final ShooterSS shooter;
    private final TurretSS  turret;
    private final LimelightLocalizerSS limelightLocalizer;
    private final PIDFController headingAimController;
    private final ShooterEquation shooterEquation = new ShooterEquation();

    // ── Commands ──────────────────────────────────────────────────────────────
    private final IntakeCommands intakeCommands;

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
    private boolean lastFailsafeToggle = false;
    private boolean lastRobotAimToggle = false;
    private boolean lastLimelightRelocalize = false;
    private boolean lastPoseResetStart = false;
    private boolean lastPoseResetCenter = false;
    private boolean turretFailsafeEnabled = false;
    private boolean robotAimEnabled = false;
    private boolean isInTuning;
    private double failsafeHeadingErrorRad = 0.0;
    private IntakeSS.BallState lastRumbledBallState = IntakeSS.BallState.EMPTY;

    // ── Constructor ───────────────────────────────────────────────────────────
    public Robot(HardwareMap hwm, Telemetry telemetry, boolean isBlueAlliance, boolean tuning) {
        this.telemetry      = telemetry;
        this.isBlueAlliance = isBlueAlliance;
        this.isInTuning     = tuning;

        follower = Constants.createFollower(hwm);
        headingAimController = new PIDFController(Constants.followerConstants.getCoefficientsHeadingPIDF());

        intake  = new IntakeSS(hwm, telemetry, "intake1", "intake2", "gate", "led1");
        shooter = new ShooterSS(hwm, telemetry, "shooter1", "shooter2", "hood", "led2");
        turret  = new TurretSS(hwm, telemetry, "turret1", "turret2");
        limelightLocalizer = new LimelightLocalizerSS(hwm, telemetry);

        intakeCommands = new IntakeCommands(intake);

        PanelsDebug.init();
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

        boolean failsafeTogglePressed = gamepad1.share && !lastFailsafeToggle;
        if (failsafeTogglePressed) {
            turretFailsafeEnabled = !turretFailsafeEnabled;
            robotAimEnabled = false;
            headingAimController.reset();
        }
        boolean robotAimTogglePressed = turretFailsafeEnabled && gamepad1.y && !lastRobotAimToggle;
        if (robotAimTogglePressed) {
            robotAimEnabled = !robotAimEnabled;
            headingAimController.reset();
        }

        // ── Pose & velocity (needed before driving block) ─────────────────────
        Pose   pose     = follower.getPose();
        double robotX   = pose.getX();
        double robotY   = pose.getY();
        double heading  = pose.getHeading();
        Vector robotVel = follower.getVelocity();

        // ── Turret & shooter (before driving so robotNeedToTurn is fresh) ──────
        if (turretFailsafeEnabled) {
            turret.failsafeParkCMD(RobotConstants.TURRET_FAILSAFE_SERVO_POS);
        } else {
            turret.aimAtTargetCMD(robotX, robotY, heading, robotVel, follower.getAngularVelocity(),
                    aimGoalX(), aimGoalY());
        }
        shooter.shooterSpinCMD(robotX, robotY, heading, robotVel,
                shootingGoalX(), shootingGoalY(), intakeCommands.isTransferring());

        // ── Driving ───────────────────────────────────────────────────────────
        double headingOffset = isBlueAlliance ? HEADING_BLUE : HEADING_RED;

        // During transfer, if the robot needs to turn to face the goal,
        // override the driver's rotation input with an auto-correction
        double turnInput = -gamepad1.right_stick_x * 0.75;
        if (robotAimEnabled) {
            turnInput = calculateFailsafeHeadingTurn(robotX, robotY, heading, robotVel);
        } else {
            headingAimController.reset();
            failsafeHeadingErrorRad = calculateFailsafeHeadingError(robotX, robotY, heading, robotVel);
        }
        if (!turretFailsafeEnabled && intakeCommands.isTransferring() && turret.robotNeedToTurn()) {
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

        boolean limelightRelocalizePressed = gamepad1.a && !lastLimelightRelocalize;
        limelightLocalizer.updateRobotOrientation(follower.getPose());
        if (limelightRelocalizePressed) {
            limelightLocalizer.setInstantReset(true);
            limelightLocalizer.relocalizeNow(follower);
        } else {
            limelightLocalizer.relocalizeIfDue(follower);
        }
        PoseStorage.setCurrentPose(follower.getPose());

        // ── Edge detection for toggle buttons ─────────────────────────────────
        boolean intakePressed    = gamepad1.right_bumper && !lastIntake;
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
        if (gamepad1.left_bumper) {
            if (!intakeCommands.isTransferring()) intakeCommands.transfer();
        } else if (intakeCommands.isTransferring()) {
            intakeCommands.idle();
        }

        // ── Save button states for next frame ─────────────────────────────────
        lastIntake    = gamepad1.right_bumper;
        lastLimelightRelocalize = gamepad1.a;
        lastFailsafeToggle = gamepad1.share;
        lastRobotAimToggle = gamepad1.y;
        lastPoseResetCenter = gamepad1.dpad_up;
        lastPoseResetStart = gamepad1.dpad_down;

        // ── Command updates ───────────────────────────────────────────────────
        boolean aimedAtTarget = turretFailsafeEnabled
                ? robotAimEnabled && isFailsafeHeadingReady(robotX, robotY, heading, robotVel)
                : !turret.robotNeedToTurn();
        intake.read();
        intakeCommands.update(
                robotX, robotY,
                shooter.isReady(),
                aimedAtTarget
        );
        rumbleOnBallDetected(gamepad1);

        // ── Subsystem updates ─────────────────────────────────────────────────
        intake.update(false);
        shooter.update(false);
        turret.update(false);
        teleopTelemetry();
        PanelsDebug.update(follower, shooter, turret, intakeCommands.isTransferring());
    }

    // ── Getters ───────────────────────────────────────────────────────────────
    public Follower getFollower()    { return follower;        }
    public boolean  isBlueAlliance() { return isBlueAlliance;  }

    private void teleopTelemetry() {
        Pose robotPose = follower.getPose();
        Pose limelightPose = limelightLocalizer.getLastVisionPose();

        telemetry.addData("Robot position", "%.1f, %.1f, %.1f",
                robotPose.getX(),
                robotPose.getY(),
                Math.toDegrees(robotPose.getHeading()));
        if (limelightPose != null) {
            telemetry.addData("Limelight pose", "%.1f, %.1f, %.1f",
                    limelightPose.getX(),
                    limelightPose.getY(),
                    Math.toDegrees(limelightPose.getHeading()));
        } else {
            telemetry.addData("Limelight pose", "none");
        }
        telemetry.addData("Intake state", intakeCommands.getState());
        telemetry.addData("Shooter at target", shooter.isReady());
        telemetry.addData("Turret failsafe", turretFailsafeEnabled);
        telemetry.addData("Turret target field deg", "%.1f", turret.getTargetAngleFieldDeg());
    }

    private double calculateFailsafeHeadingTurn(double robotX, double robotY, double heading, Vector robotVel) {
        failsafeHeadingErrorRad = calculateFailsafeHeadingError(robotX, robotY, heading, robotVel);
        headingAimController.updateError(failsafeHeadingErrorRad);
        return Range.clip(
                headingAimController.run(),
                -RobotConstants.TURRET_FAILSAFE_MAX_TURN_POWER,
                RobotConstants.TURRET_FAILSAFE_MAX_TURN_POWER
        );
    }

    private void rumbleOnBallDetected(Gamepad gamepad1) {
        IntakeSS.BallState ballState = intakeCommands.getBallState();
        if (!intakeCommands.isIntaking()) {
            lastRumbledBallState = IntakeSS.BallState.EMPTY;
            return;
        }

        if (ballState == lastRumbledBallState) {
            return;
        }

        if (ballState == IntakeSS.BallState.ONE) {
            gamepad1.rumble(0.45, 0.45, 200);
        } else if (ballState == IntakeSS.BallState.FULL) {
            gamepad1.rumble(1.0, 1.0, 350);
        }

        lastRumbledBallState = ballState;
    }

    private boolean isFailsafeHeadingReady(double robotX, double robotY, double heading, Vector robotVel) {
        failsafeHeadingErrorRad = calculateFailsafeHeadingError(robotX, robotY, heading, robotVel);
        return Math.abs(failsafeHeadingErrorRad) <= RobotConstants.TURRET_FAILSAFE_AIM_TOLERANCE_RAD;
    }

    private double calculateFailsafeHeadingError(double robotX, double robotY, double heading, Vector robotVel) {
        double targetHeading = calculateFailsafeTargetHeading(robotX, robotY, heading, robotVel);
        return normalizeAngle(targetHeading - heading);
    }

    private double calculateFailsafeTargetHeading(double robotX, double robotY, double heading, Vector robotVel) {
        if (robotVel == null) {
            robotVel = new Vector(0, 0);
        }

        double turretFieldX = robotX + TURRET_OFFSET_X * Math.cos(heading)
                - TURRET_OFFSET_Y * Math.sin(heading);
        double turretFieldY = robotY + TURRET_OFFSET_X * Math.sin(heading)
                + TURRET_OFFSET_Y * Math.cos(heading);

        double distance = Math.hypot(aimGoalX() - turretFieldX, aimGoalY() - turretFieldY);
        double airtime = shooterEquation.getAirTime(distance);
        double adjustedGoalX = aimGoalX() - robotVel.getXComponent() * airtime;
        double adjustedGoalY = aimGoalY() - robotVel.getYComponent() * airtime;

        double turretFieldTargetAngle = Math.atan2(
                adjustedGoalY - turretFieldY,
                adjustedGoalX - turretFieldX
        );

        return normalizeAngle(turretFieldTargetAngle + Math.PI
                + RobotConstants.TURRET_FAILSAFE_HEADING_OFFSET_RAD);
    }

    private void resetPose(Pose pose) {
        if (PoseStorage.isValid(pose)) {
            follower.setPose(pose);
            PoseStorage.setCurrentPose(pose);
        }
    }

    private static double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
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
