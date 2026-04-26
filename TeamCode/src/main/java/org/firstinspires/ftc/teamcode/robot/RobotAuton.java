package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.PtoSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;

import java.util.List;

public class RobotAuton {

    // ── Subsystems ────────────────────────────────────────────────────────────
    public final Follower  follower;
    public final IntakeSS  intake;
    public final ShooterSS shooter;
    public final TurretSS  turret;
    private final PtoSS    pto; // private — no lift access in auton

    // ── Commands ──────────────────────────────────────────────────────────────
    public final IntakeCommands intakeCommands;

    // ── Bulk caching ──────────────────────────────────────────────────────────
    private final List<LynxModule> hubs;

    // ── Alliance ──────────────────────────────────────────────────────────────
    private final boolean isBlueAlliance;

    public final double goalX;
    public final double goalY;

    // ── State ─────────────────────────────────────────────────────────────────
    private State currentState = State.IDLE;

    private enum State {
        IDLE,
        INTAKING,           // intake with timeout
        TRANSFERRING,       // transfer with timeout
        DRIVE_AND_INTAKE,   // follow path + intake, exit when path done
        DRIVE_AND_INTAKE_EARLY, // follow path + intake, exit when both balls detected
        PATH_AND_TRANSFER   // follow path, then auto-transfer on arrival
    }

    // ── Timed action ──────────────────────────────────────────────────────────
    private final ElapsedTime actionTimer   = new ElapsedTime();
    private double             actionTimeoutMs = 0.0;

    // ── Constructor ───────────────────────────────────────────────────────────
    public RobotAuton(HardwareMap hwm, Telemetry telemetry, boolean isBlueAlliance) {
        this.isBlueAlliance = isBlueAlliance;

        goalX = isBlueAlliance ? RobotConstants.GOAL_X_BLUE : RobotConstants.GOAL_X_RED;
        goalY = isBlueAlliance ? RobotConstants.GOAL_Y_BLUE : RobotConstants.GOAL_Y_RED;

        // Bulk REV Hub caching
        hubs = hwm.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.createFollower(hwm);

        intake  = new IntakeSS(hwm, telemetry,
                RobotConstants.INTAKE_MOTOR_1, RobotConstants.INTAKE_MOTOR_2,
                RobotConstants.GATE_SERVO,     RobotConstants.INTAKE_LED);
        shooter = new ShooterSS(hwm, telemetry,
                RobotConstants.SHOOTER_MOTOR_1, RobotConstants.SHOOTER_MOTOR_2,
                RobotConstants.HOOD_SERVO,      RobotConstants.SHOOTER_LED);
        turret  = new TurretSS(hwm, telemetry,
                RobotConstants.TURRET_SERVO_1, RobotConstants.TURRET_SERVO_2);
        pto     = new PtoSS(hwm, telemetry,
                RobotConstants.PTO_SERVO,
                RobotConstants.FRONT_LEFT,  RobotConstants.FRONT_RIGHT,
                RobotConstants.BACK_LEFT,   RobotConstants.BACK_RIGHT);

        intakeCommands = new IntakeCommands(intake);

        // Safety: force PTO disengaged at startup — three layers of protection:
        // 1. disengagePtoCMD() sets the state
        // 2. write() pushes it to hardware immediately
        // 3. update() calls disengagePtoCMD() every frame in the loop
        pto.disengagePtoCMD();
        pto.write();
    }

    // ── Robot start ───────────────────────────────────────────────────────────
    public void start(Pose startingPose) {
        follower.setStartingPose(startingPose);
    }

    // ── Main auton loop ───────────────────────────────────────────────────────

    /**
     * Call every loop unconditionally — handles all subsystems, the state
     * machine, and command updates regardless of whether isBusy() is true.
     */
    public void update() {
        // Clear bulk cache at top of every loop
        for (LynxModule hub : hubs) hub.clearBulkCache();

        follower.update();

        // Force PTO disengaged every frame — lift is locked out in auton
        pto.disengagePtoCMD();

        // ── Pose & velocity ───────────────────────────────────────────────────
        Pose   pose     = follower.getPose();
        double robotX   = pose.getX();
        double robotY   = pose.getY();
        double heading  = pose.getHeading();
        Vector robotVel = follower.getVelocity();

        // ── Turret & shooter (automatic, always running) ──────────────────────
        turret.aimAtTargetCMD(robotX, robotY, heading, robotVel,
                follower.getAngularVelocity(), goalX, goalY);
        shooter.shooterSpinCMD(robotX, robotY, heading, robotVel, goalX, goalY);

        // ── State machine ─────────────────────────────────────────────────────
        switch (currentState) {

            case INTAKING:
                if (actionTimer.milliseconds() >= actionTimeoutMs) {
                    intakeCommands.idle();
                    currentState = State.IDLE;
                }
                break;

            case TRANSFERRING:
                boolean transferComplete = !intakeCommands.isTransferring();
                boolean timedOut        = actionTimer.milliseconds() >= actionTimeoutMs;
                if (transferComplete || timedOut) {
                    intakeCommands.idle();
                    currentState = State.IDLE;
                }
                break;

            case DRIVE_AND_INTAKE:
                // Exit when path finishes
                if (!follower.isBusy()) {
                    intakeCommands.idle();
                    currentState = State.IDLE;
                }
                break;

            case DRIVE_AND_INTAKE_EARLY:
                // Exit early when both balls detected, or when path finishes
                if (intakeCommands.isFullyLoaded() || !follower.isBusy()) {
                    intakeCommands.idle();
                    follower.breakFollowing(); // stop immediately if balls detected early
                    currentState = State.IDLE;
                }
                break;

            case PATH_AND_TRANSFER:
                // Wait for path to finish, then automatically start transferring
                if (!follower.isBusy()) {
                    intakeCommands.transfer();
                    actionTimer.reset();
                    currentState = State.TRANSFERRING;
                }
                break;

            case IDLE:
            default:
                break;
        }

        // ── Command & subsystem updates (always run) ──────────────────────────
        intakeCommands.update(
            robotX, robotY,
            shooter.isReady(),
            !turret.robotNeedToTurn()
        );

        intake.update();
        shooter.update();
        turret.update();
        pto.update(); // pushes disengagedPos set above
    }

    // ── Auton action commands ─────────────────────────────────────────────────

    /**
     * Intake for up to timeoutMs ms.
     * isBusy() returns true until timeout expires.
     */
    public void intakeFor(double timeoutMs) {
        intakeCommands.intake();
        actionTimeoutMs = timeoutMs;
        currentState    = State.INTAKING;
        actionTimer.reset();
    }

    /**
     * Transfer for up to timeoutMs ms.
     * isBusy() returns true until transfer completes or timeout expires.
     */
    public void transferFor(double timeoutMs) {
        intakeCommands.transfer();
        actionTimeoutMs = timeoutMs;
        currentState    = State.TRANSFERRING;
        actionTimer.reset();
    }

    /**
     * Follow a path while intaking simultaneously.
     * isBusy() returns true until the path finishes.
     *
     * @param path the PedroPathing path to follow
     */
    public void driveAndIntake(Path path) {
        follower.followPath(path, true);
        intakeCommands.intake();
        currentState = State.DRIVE_AND_INTAKE;
        actionTimer.reset();
    }

    /**
     * Follow a path while intaking simultaneously.
     * Exits EARLY and stops the follower immediately if both balls are detected
     * before the path finishes, saving time.
     *
     * @param path the PedroPathing path to follow
     */
    public void driveAndIntakeEarlyExit(Path path) {
        follower.followPath(path, true);
        intakeCommands.intake();
        currentState = State.DRIVE_AND_INTAKE_EARLY;
        actionTimer.reset();
    }

    /**
     * Follow a path, then automatically begin transferring on arrival.
     * isBusy() returns true through the entire path + transfer sequence.
     *
     * @param path              the PedroPathing path to follow
     * @param transferTimeoutMs safety timeout for the transfer phase (ms)
     */
    public void pathAndTransfer(Path path, double transferTimeoutMs) {
        follower.followPath(path, true);
        actionTimeoutMs = transferTimeoutMs;
        currentState    = State.PATH_AND_TRANSFER;
        actionTimer.reset();
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    /**
     * Returns true while any action is running.
     * Always call update() every loop regardless of isBusy().
     */
    public boolean isBusy() {
        return currentState != State.IDLE;
    }

    public Follower getFollower()    { return follower;       }
    public boolean  isBlueAlliance() { return isBlueAlliance; }
}
