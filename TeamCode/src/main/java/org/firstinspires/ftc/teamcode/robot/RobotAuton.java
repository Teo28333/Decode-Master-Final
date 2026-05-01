package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.LimelightLocalizerSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;

public class RobotAuton {

    // ── Subsystems ────────────────────────────────────────────────────────────
    public final Follower  follower;
    public final IntakeSS  intake;
    public final ShooterSS shooter;
    public final TurretSS  turret;
    public final LimelightLocalizerSS limelightLocalizer;

    // ── Commands ──────────────────────────────────────────────────────────────
    public final IntakeCommands intakeCommands;

    // ── Alliance ──────────────────────────────────────────────────────────────
    private final boolean isBlueAlliance;

    // ── Goal position ─────────────────────────────────────────────────────────
    // ── Zero velocity fallback ────────────────────────────────────────────────
    private static final Vector ZERO_VEL = new Vector(0, 0);

    // ── State ─────────────────────────────────────────────────────────────────
    private State currentState = State.IDLE;

    private enum State {
        IDLE,
        INTAKING,
        TRANSFERRING,
        FOLLOWING,
        FOLLOWING_AND_INTAKE
    }

    // ── Timed action ──────────────────────────────────────────────────────────
    private final ElapsedTime actionTimer   = new ElapsedTime();
    private double             actionTimeoutMs = 0.0;
    private double             pathIntakeTimeoutMs = -1.0;

    // ── Constructor ───────────────────────────────────────────────────────────
    public RobotAuton(HardwareMap hwm, Telemetry telemetry, boolean isBlueAlliance) {
        this.isBlueAlliance = isBlueAlliance;

        follower = Constants.createFollower(hwm);

        intake  = new IntakeSS(hwm, telemetry, "intake1", "intake2", "gate", "led1");
        shooter = new ShooterSS(hwm, telemetry, "shooter1", "shooter2", "hood", "led2");
        turret  = new TurretSS(hwm, telemetry, "turret1", "turret2");
        limelightLocalizer = new LimelightLocalizerSS(hwm, telemetry);

        intakeCommands = new IntakeCommands(intake);
        intake.setIgnoreShootingZoneCheck(true);

        PanelsDebug.init();
    }

    // ── Robot start ───────────────────────────────────────────────────────────
    public void start(Pose startingPose) {
        if (startingPose == null) {
            throw new IllegalArgumentException("RobotAuton startingPose cannot be null");
        }

        follower.setStartingPose(startingPose);
    }

    // ── Main auton loop ───────────────────────────────────────────────────────
    public void update() {
        follower.update();
        limelightLocalizer.updateRobotOrientation(follower.getPose());
        limelightLocalizer.relocalizeIfDue(follower);

        // ── Pose & velocity ───────────────────────────────────────────────────
        Pose   pose       = follower.getPose();
        double robotX     = pose.getX();
        double robotY     = pose.getY();
        double heading    = pose.getHeading();
        Vector robotVel   = follower.getVelocity();
        double angularVel = follower.getAngularVelocity();

        if (robotVel == null) robotVel = ZERO_VEL;

        // ── Turret & shooter — always running ─────────────────────────────────
        turret.aimAtTargetCMD(robotX, robotY, heading, robotVel, angularVel,
                aimGoalX(), aimGoalY());
        shooter.shooterSpinCMD(robotX, robotY, heading, robotVel,
                shootingGoalX(), shootingGoalY(), intakeCommands.isTransferring());

        // ── State machine ─────────────────────────────────────────────────────
        switch (currentState) {

            case INTAKING:
                if (actionTimer.milliseconds() >= actionTimeoutMs) {
                    intakeCommands.idle();
                    currentState = State.IDLE;
                }
                break;

            case TRANSFERRING:
                boolean timedOut = actionTimer.milliseconds() >= actionTimeoutMs;
                if (timedOut) {
                    intakeCommands.idle();
                    currentState = State.IDLE;
                } else {
                    intakeCommands.transfer();
                }
                break;

            case FOLLOWING:
                if (!follower.isBusy()) {
                    currentState = State.IDLE;
                }
                break;

            case FOLLOWING_AND_INTAKE:
                if (pathIntakeTimeoutMs >= 0.0 && actionTimer.milliseconds() >= pathIntakeTimeoutMs) {
                    intakeCommands.idle();
                    pathIntakeTimeoutMs = -1.0;
                }

                if (!follower.isBusy()) {
                    intakeCommands.idle();
                    pathIntakeTimeoutMs = -1.0;
                    currentState = State.IDLE;
                }
                break;

            case IDLE:
            default:
                break;
        }

        // ── Command & subsystem updates ───────────────────────────────────────
        intakeCommands.update(
                robotX, robotY,
                shooter.isReady(),
                !turret.robotNeedToTurn()
        );

        intake.update();
        shooter.update();
        turret.update();
        limelightLocalizer.telemetry();
        PanelsDebug.update(follower, shooter, turret, intakeCommands.isTransferring());
        savePose();
    }

    // ── Commands ──────────────────────────────────────────────────────────────

    /** Intake for up to timeoutMs ms. */
    public void intakeFor(double timeoutMs) {
        intakeCommands.intake();
        actionTimeoutMs = timeoutMs;
        currentState    = State.INTAKING;
        actionTimer.reset();
    }

    /** Transfer for up to timeoutMs ms. Auto-exits when shot completes. */
    public void transferFor(double timeoutMs) {
        intakeCommands.transfer();
        actionTimeoutMs = timeoutMs;
        currentState    = State.TRANSFERRING;
        actionTimer.reset();
    }

    /** Follow a path. isBusy() returns true until the path finishes. */
    public void followPath(Path path) {
        follower.followPath(path, true);
        currentState = State.FOLLOWING;
    }

    /** Follow a path with optional end hold. */
    public void followPath(Path path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        currentState = State.FOLLOWING;
    }

    /** Follow a path chain. isBusy() returns true until the path finishes. */
    public void followPath(PathChain path) {
        follower.followPath(path, true);
        currentState = State.FOLLOWING;
    }

    /** Follow a path chain with optional end hold. */
    public void followPath(PathChain path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        currentState = State.FOLLOWING;
    }

    /**
     * Follow a path while intaking simultaneously.
     * isBusy() returns true until the path finishes.
     */
    public void followPathAndIntake(Path path) {
        follower.followPath(path, true);
        intakeCommands.intake();
        pathIntakeTimeoutMs = -1.0;
        currentState = State.FOLLOWING_AND_INTAKE;
    }

    /**
     * Follow a path while intaking simultaneously, with optional end hold.
     * isBusy() returns true until the path finishes.
     */
    public void followPathAndIntake(Path path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        intakeCommands.intake();
        pathIntakeTimeoutMs = -1.0;
        currentState = State.FOLLOWING_AND_INTAKE;
    }

    /**
     * Follow a path while intaking only for the first intakeTimeoutMs.
     * The path keeps running after the intake turns off.
     */
    public void followPathAndIntakeFor(Path path, double intakeTimeoutMs) {
        followPathAndIntakeFor(path, intakeTimeoutMs, true);
    }

    /**
     * Follow a path while intaking only for the first intakeTimeoutMs, with optional end hold.
     */
    public void followPathAndIntakeFor(Path path, double intakeTimeoutMs, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        intakeCommands.intake();
        pathIntakeTimeoutMs = intakeTimeoutMs;
        currentState = State.FOLLOWING_AND_INTAKE;
        actionTimer.reset();
    }

    /**
     * Follow a path chain while intaking simultaneously.
     * isBusy() returns true until the path finishes.
     */
    public void followPathAndIntake(PathChain path) {
        follower.followPath(path, true);
        intakeCommands.intake();
        pathIntakeTimeoutMs = -1.0;
        currentState = State.FOLLOWING_AND_INTAKE;
    }

    /**
     * Follow a path chain while intaking simultaneously, with optional end hold.
     * isBusy() returns true until the path finishes.
     */
    public void followPathAndIntake(PathChain path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        intakeCommands.intake();
        pathIntakeTimeoutMs = -1.0;
        currentState = State.FOLLOWING_AND_INTAKE;
    }

    /**
     * Follow a path chain while intaking only for the first intakeTimeoutMs.
     * The path keeps running after the intake turns off.
     */
    public void followPathAndIntakeFor(PathChain path, double intakeTimeoutMs) {
        followPathAndIntakeFor(path, intakeTimeoutMs, true);
    }

    /**
     * Follow a path chain while intaking only for the first intakeTimeoutMs, with optional end hold.
     */
    public void followPathAndIntakeFor(PathChain path, double intakeTimeoutMs, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        intakeCommands.intake();
        pathIntakeTimeoutMs = intakeTimeoutMs;
        currentState = State.FOLLOWING_AND_INTAKE;
        actionTimer.reset();
    }

    // ── Getters ───────────────────────────────────────────────────────────────
    public boolean isBusy()         { return currentState != State.IDLE; }
    public boolean isShooterReady() { return shooter.isReady();          }
    public Follower getFollower()    { return follower;                   }
    public boolean  isBlueAlliance() { return isBlueAlliance;             }

    public void forceIdle() {
        follower.breakFollowing();
        intakeCommands.idle();
        currentState = State.IDLE;
        pathIntakeTimeoutMs = -1.0;
    }

    public double getPathProgressPercent() {
        if (!follower.isBusy()) {
            return 100.0;
        }

        return Math.max(0.0, Math.min(100.0, follower.getPathCompletion() * 100.0));
    }

    public void savePose() {
        PoseStorage.setCurrentPose(follower.getPose());
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
