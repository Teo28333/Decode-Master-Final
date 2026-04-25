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
    private static final double HEADING_BLUE = Math.PI;

    // ── Goal position ─────────────────────────────────────────────────────────
    private static final double GOAL_X_RED  = 142.0;
    private static final double GOAL_Y_RED  = 144.0;
    private static final double GOAL_X_BLUE = 2.0;
    private static final double GOAL_Y_BLUE = 144.0;

    private final double goalX;
    private final double goalY;

    // ── Button edge detection ─────────────────────────────────────────────────
    private boolean lastIntake    = false;
    private boolean lastLift      = false;
    private boolean lastDisengage = false;
    private boolean isInTuning;

    // ── Constructor ───────────────────────────────────────────────────────────
    public Robot(HardwareMap hwm, Telemetry telemetry, boolean isBlueAlliance, boolean tuning) {
        this.isBlueAlliance = isBlueAlliance;
        this.isInTuning = tuning;

        goalX = isBlueAlliance ? GOAL_X_BLUE : GOAL_X_RED;
        goalY = isBlueAlliance ? GOAL_Y_BLUE : GOAL_Y_RED;

        follower = Constants.createFollower(hwm);

        intake  = new IntakeSS(hwm, telemetry, "intake1", "intake2", "gate", "led1");
        shooter = new ShooterSS(hwm, telemetry, "shooter1", "shooter2", "hood", "led2");
        turret  = new TurretSS(hwm, telemetry, "turret1", "turret2");
        pto     = new PtoSS(hwm, telemetry, "pto",
                "frontLeft", "frontRight", "backLeft", "backRight");

        intakeCommands = new IntakeCommands(intake);
        liftCommands   = new LiftCommands(pto);

        // Safety: force PTO disengaged at startup before any loop runs
        pto.disengagePtoCMD();
        pto.write();
    }

    // ── Robot start ───────────────────────────────────────────────────────────
    public void start() {
        follower.setStartingPose(new Pose(72, 72, 0));
        follower.startTeleopDrive();
    }

    // ── Main teleop loop ──────────────────────────────────────────────────────
    public void update(Gamepad gamepad1) {

        shooter.setTuningMode(isInTuning);
        // ── Driving ───────────────────────────────────────────────────────────
        double headingOffset = isBlueAlliance ? HEADING_BLUE : HEADING_RED;

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                false,
                Math.toRadians(headingOffset)
        );
        follower.update();

        // ── Pose & velocity ───────────────────────────────────────────────────
        Pose pose = follower.getPose();
        double robotX   = pose.getX();
        double robotY   = pose.getY();
        double heading  = pose.getHeading();
        Vector robotVel = follower.getVelocity();

        // ── Turret & shooter (automatic) ──────────────────────────────────────
        turret.aimAtTargetCMD(robotX, robotY, heading, robotVel, goalX, goalY);
        shooter.shooterSpinCMD(robotX, robotY, heading, robotVel, goalX, goalY);

        // ── Edge detection for toggle buttons ─────────────────────────────────
        boolean intakePressed    = gamepad1.right_bumper && !lastIntake;
        boolean liftPressed      = gamepad1.y            && !lastLift;
        boolean disengagePressed = gamepad1.left_bumper  && !lastDisengage;

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

        // ── Transfer (active while X held) ────────────────────────────────────
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
}