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
import org.firstinspires.ftc.teamcode.subsystems.PtoSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;

public class RobotAuton {

    // ── Subsystems ────────────────────────────────────────────────────────────
    public final Follower  follower;
    public final IntakeSS  intake;
    public final ShooterSS shooter;
    public final TurretSS  turret;
    private final PtoSS    pto;

    // ── Commands ──────────────────────────────────────────────────────────────
    public final IntakeCommands intakeCommands;

    // ── Alliance ──────────────────────────────────────────────────────────────
    private final boolean isBlueAlliance;

    // ── Goal position ─────────────────────────────────────────────────────────
    private static final double GOAL_X_RED  = 136.0;
    private static final double GOAL_Y_RED  = 136.0;
    private static final double GOAL_X_BLUE = 8.0;
    private static final double GOAL_Y_BLUE = 136.0;

    public final double goalX;
    public final double goalY;

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

    // ── Constructor ───────────────────────────────────────────────────────────
    public RobotAuton(HardwareMap hwm, Telemetry telemetry, boolean isBlueAlliance) {
        this.isBlueAlliance = isBlueAlliance;

        goalX = isBlueAlliance ? GOAL_X_BLUE : GOAL_X_RED;
        goalY = isBlueAlliance ? GOAL_Y_BLUE : GOAL_Y_RED;

        follower = Constants.createFollower(hwm);

        intake  = new IntakeSS(hwm, telemetry, "intake1", "intake2", "gate", "led1");
        shooter = new ShooterSS(hwm, telemetry, "shooter1", "shooter2", "hood", "led2");
        turret  = new TurretSS(hwm, telemetry, "turret1", "turret2");
        pto     = new PtoSS(hwm, telemetry, "pto",
                "frontLeft", "frontRight", "backLeft", "backRight");

        intakeCommands = new IntakeCommands(intake);
        intake.setIgnoreShootingZoneCheck(true);

        disengagePto();
    }

    // ── Robot start ───────────────────────────────────────────────────────────
    public void start(Pose startingPose) {
        if (startingPose == null) {
            throw new IllegalArgumentException("RobotAuton startingPose cannot be null");
        }

        follower.setStartingPose(startingPose);
    }

    public void disengagePto() {
        pto.disengagePtoCMD();
        pto.write();
    }

    // ── Main auton loop ───────────────────────────────────────────────────────
    public void update() {
        follower.update();

        pto.disengagePtoCMD();

        // ── Pose & velocity ───────────────────────────────────────────────────
        Pose   pose       = follower.getPose();
        double robotX     = pose.getX();
        double robotY     = pose.getY();
        double heading    = pose.getHeading();
        Vector robotVel   = follower.getVelocity();
        double angularVel = follower.getAngularVelocity();

        if (robotVel == null) robotVel = ZERO_VEL;

        // ── Turret & shooter — always running ─────────────────────────────────
        turret.aimAtTargetCMD(robotX, robotY, heading, robotVel, angularVel, goalX, goalY);
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
                if (!follower.isBusy()) {
                    intakeCommands.idle();
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
        pto.update();
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

    /** Follow a path chain. isBusy() returns true until the path finishes. */
    public void followPath(PathChain path) {
        follower.followPath(path, true);
        currentState = State.FOLLOWING;
    }

    /**
     * Follow a path while intaking simultaneously.
     * isBusy() returns true until the path finishes.
     */
    public void followPathAndIntake(Path path) {
        follower.followPath(path, true);
        intakeCommands.intake();
        currentState = State.FOLLOWING_AND_INTAKE;
    }

    /**
     * Follow a path chain while intaking simultaneously.
     * isBusy() returns true until the path finishes.
     */
    public void followPathAndIntake(PathChain path) {
        follower.followPath(path, true);
        intakeCommands.intake();
        currentState = State.FOLLOWING_AND_INTAKE;
    }

    // ── Getters ───────────────────────────────────────────────────────────────
    public boolean isBusy()         { return currentState != State.IDLE; }
    public boolean isShooterReady() { return shooter.isReady();          }
    public Follower getFollower()    { return follower;                   }
    public boolean  isBlueAlliance() { return isBlueAlliance;             }
}
