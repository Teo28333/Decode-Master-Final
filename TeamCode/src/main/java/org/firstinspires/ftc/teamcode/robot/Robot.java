package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
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

import java.util.List;

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

    // ── Gamepads ──────────────────────────────────────────────────────────────

    private GamepadWrapper gp;

    // ── Bulk caching ──────────────────────────────────────────────────────────
    private final List<LynxModule> hubs;

    // ── Alliance ──────────────────────────────────────────────────────────────
    private final boolean isBlueAlliance;

    private final double goalX;
    private final double goalY;

    // ── Slew rate limiter ─────────────────────────────────────────────────────
    // Limits how fast drive power can change to prevent current spikes.
    // SLEW_RATE: max power change per second.
    //   2.0 = full range (0→1) in 0.5s  — smooth, good for heavy robots
    //   4.0 = full range in 0.25s        — snappier, tune up if too sluggish
    //   8.0 = full range in 0.125s       — fast, minimal protection
    private static final double SLEW_RATE = 2.0;

    private double slewX      = 0.0;
    private double slewY      = 0.0;
    private double slewTurn   = 0.0;
    private long   lastLoopNs = -1L;

    // ── Constructor ───────────────────────────────────────────────────────────
    public Robot(HardwareMap hwm, Telemetry telemetry, boolean isBlueAlliance, boolean tuning) {
        this.isBlueAlliance = isBlueAlliance;

        goalX = isBlueAlliance ? RobotConstants.GOAL_X_BLUE : RobotConstants.GOAL_X_RED;
        goalY = isBlueAlliance ? RobotConstants.GOAL_Y_BLUE : RobotConstants.GOAL_Y_RED;

        // Bulk REV Hub caching — clears every loop for fresh sensor reads
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
        liftCommands   = new LiftCommands(pto);

        shooter.setTuningMode(tuning);

        // Safety: force PTO disengaged at startup before any loop runs
        pto.disengagePtoCMD();
        pto.write();
    }

    // ── Robot start ───────────────────────────────────────────────────────────
    public void start(Gamepad gamepad1) {
        this.gp = new GamepadWrapper(gamepad1);
        follower.setStartingPose(new Pose(
                isBlueAlliance ? RobotConstants.START_X_BLUE : RobotConstants.START_X_RED,
                isBlueAlliance ? RobotConstants.START_Y_BLUE : RobotConstants.START_Y_RED,
                isBlueAlliance ? RobotConstants.HEADING_BLUE : RobotConstants.HEADING_RED
        ));
        follower.startTeleopDrive();
    }

    // ── Main teleop loop ──────────────────────────────────────────────────────
    public void update(Gamepad gamepad1) {
        // Clear bulk cache at top of every loop
        for (LynxModule hub : hubs) hub.clearBulkCache();

        // Wrap gamepad — update() must be called before reading any buttons
        gp.update();

        // ── Loop timing for slew rate ─────────────────────────────────────────
        long   nowNs = System.nanoTime();
        double dt    = lastLoopNs < 0 ? 0.02 : (nowNs - lastLoopNs) * 1e-9;
        lastLoopNs   = nowNs;

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

        // ── Raw driver inputs ─────────────────────────────────────────────────
        double headingOffset = isBlueAlliance
                ? RobotConstants.HEADING_BLUE
                : RobotConstants.HEADING_RED;

        double rawTurn = -gp.rightStickX();
        if (intakeCommands.isTransferring() && turret.robotNeedToTurn()) {
            rawTurn = Math.copySign(RobotConstants.TURN_CORRECTION_POWER,
                    turret.getTurretAngleDeg());
        }

        double rawX = -gp.leftStickX() * RobotConstants.DRIVE_SPEED_MULTIPLIER;
        double rawY = -gp.leftStickY() * RobotConstants.DRIVE_SPEED_MULTIPLIER;

        // ── Slew rate limiting ────────────────────────────────────────────────
        // Caps the rate of change of each axis to SLEW_RATE per second.
        // This smooths out sudden direction changes and startup spikes.
        double maxDelta = SLEW_RATE * dt;
        slewX    += clamp(rawX    - slewX,    -maxDelta, maxDelta);
        slewY    += clamp(rawY    - slewY,    -maxDelta, maxDelta);
        slewTurn += clamp(rawTurn - slewTurn, -maxDelta, maxDelta);

        follower.setTeleOpDrive(slewY, slewX, slewTurn, false, Math.toRadians(headingOffset));
        follower.update();

        // ── Pose reset ────────────────────────────────────────────────────────
        if (gp.justPressed(GamepadWrapper.Button.SHARE)) {
            follower.setPose(new Pose(
                    isBlueAlliance ? RobotConstants.START_X_BLUE : RobotConstants.START_X_RED,
                    isBlueAlliance ? RobotConstants.START_Y_BLUE : RobotConstants.START_Y_RED,
                    isBlueAlliance ? RobotConstants.HEADING_BLUE : RobotConstants.HEADING_RED
            ));
        }

        // ── Intake (toggle on right bumper) ───────────────────────────────────
        if (gp.justPressed(GamepadWrapper.Button.RIGHT_BUMPER)) {
            if (intakeCommands.isIntaking()) intakeCommands.idle();
            else                             intakeCommands.intake();
        }

        // ── Outtake (active while B held, idle when released) ─────────────────
        if (gp.held(GamepadWrapper.Button.B)) {
            if (!intakeCommands.isOuttaking()) intakeCommands.outtake();
        } else if (intakeCommands.isOuttaking()) {
            intakeCommands.idle();
        }

        // ── Transfer (active while X held, idle when released) ────────────────
        if (gp.held(GamepadWrapper.Button.X)) {
            if (!intakeCommands.isTransferring()) intakeCommands.transfer();
        } else if (intakeCommands.isTransferring()) {
            intakeCommands.idle();
        }

        // ── Lift (Y = engage, left bumper = disengage) ────────────────────────
        if (gp.justPressed(GamepadWrapper.Button.Y))           liftCommands.lift();
        if (gp.justPressed(GamepadWrapper.Button.LEFT_BUMPER)) liftCommands.disengage();

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
    }

    // ── Getters ───────────────────────────────────────────────────────────────
    public Follower getFollower()    { return follower;        }
    public boolean  isBlueAlliance() { return isBlueAlliance;  }

    // ── Utility ───────────────────────────────────────────────────────────────
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}