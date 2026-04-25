package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
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

    private static final double HEADING_RED  = 0.0;
    private static final double HEADING_BLUE = Math.PI;

    // ── Goal position ─────────────────────────────────────────────────────────
    private static final double GOAL_X_RED  = 142.0;
    private static final double GOAL_Y_RED  = 144.0;
    private static final double GOAL_X_BLUE = 2.0;
    private static final double GOAL_Y_BLUE = 144.0;

    public final double goalX;
    public final double goalY;

    // ── State ─────────────────────────────────────────────────────────────────
    private State currentState = State.IDLE;

    private enum State {
        IDLE,
        INTAKING,
        TRANSFERRING
    }

    // ── Timed action ──────────────────────────────────────────────────────────
    private final ElapsedTime actionTimer  = new ElapsedTime();
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

        // Safety: force PTO disengaged at startup
        pto.disengagePtoCMD();
        pto.write();
    }

    // ── Robot start ───────────────────────────────────────────────────────────
    public void start(Pose startingPose) {
        follower.setStartingPose(startingPose);
    }

    // ── Main auton loop ───────────────────────────────────────────────────────

    /**
     * Call every loop unconditionally — handles all subsystems, state machine,
     * and command updates regardless of whether isBusy() is true or false.
     */
    public void update() {
        follower.update();

        // Force PTO disengaged every frame — no lift allowed in auton
        pto.disengagePtoCMD();

        // ── Pose & velocity ───────────────────────────────────────────────────
        Pose   pose     = follower.getPose();
        double robotX   = pose.getX();
        double robotY   = pose.getY();
        double heading  = pose.getHeading();
        Vector robotVel = follower.getVelocity();

        // ── Turret & shooter (automatic) ──────────────────────────────────────
        turret.aimAtTargetCMD(robotX, robotY, heading, robotVel, follower.getAngularVelocity(), goalX, goalY);
        shooter.shooterSpinCMD(robotX, robotY, heading, robotVel, goalX, goalY);

        // ── State machine — runs every loop, transitions to IDLE when done ────
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

            case IDLE:
            default:
                break;
        }

        // ── Command & subsystem updates — always run ──────────────────────────
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

    // ── Timed action commands ─────────────────────────────────────────────────

    /**
     * Start intaking for up to timeoutMs milliseconds.
     * Robot stays busy until timeout expires.
     */
    public void intakeFor(double timeoutMs) {
        intakeCommands.intake();
        actionTimeoutMs = timeoutMs;
        currentState    = State.INTAKING;
        actionTimer.reset();
    }

    /**
     * Start transferring for up to timeoutMs milliseconds.
     * Robot stays busy until transfer completes or timeout expires.
     */
    public void transferFor(double timeoutMs) {
        intakeCommands.transfer();
        actionTimeoutMs = timeoutMs;
        currentState    = State.TRANSFERRING;
        actionTimer.reset();
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    /**
     * Returns true while an action is running.
     * Always call update() every loop regardless of isBusy().
     */
    public boolean isBusy() {
        return currentState != State.IDLE;
    }

    public Follower getFollower()    { return follower;       }
    public boolean  isBlueAlliance() { return isBlueAlliance; }
}