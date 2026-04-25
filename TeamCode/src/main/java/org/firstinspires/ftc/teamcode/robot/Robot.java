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
    private final boolean isRedAlliance;

    private static final double HEADING_RED  = 0.0;     // tune per field setup
    private static final double HEADING_BLUE = Math.PI; // 180° mirror

    // ── Goal position ─────────────────────────────────────────────────────────
    private static final double GOAL_X_RED  = 12.0;  // inches — tune per field
    private static final double GOAL_Y_RED  = 120.0;
    private static final double GOAL_X_BLUE = 132.0;
    private static final double GOAL_Y_BLUE = 120.0;

    private final double goalX;
    private final double goalY;

    // ── Button edge detection ─────────────────────────────────────────────────
    private boolean lastIntake    = false;
    private boolean lastOuttake   = false;
    private boolean lastTransfer  = false;
    private boolean lastLift      = false;
    private boolean lastDisengage = false;

    // ── Constructor ───────────────────────────────────────────────────────────
    public Robot(HardwareMap hwm, Telemetry telemetry, boolean isRedAlliance) {
        this.isRedAlliance = isRedAlliance;

        goalX = isRedAlliance ? GOAL_X_RED  : GOAL_X_BLUE;
        goalY = isRedAlliance ? GOAL_Y_RED  : GOAL_Y_BLUE;

        // Drivetrain — starting heading mirrors alliance side
        follower = Constants.createFollower(hwm);

        // Subsystems — update hardware names to match your config
        intake  = new IntakeSS(hwm, telemetry, "intake1", "intake2", "gate", "led1");
        shooter = new ShooterSS(hwm, telemetry, "shooter1", "shooter2", "hood", "led2");
        turret  = new TurretSS(hwm, telemetry, "turret1", "turret2");
        pto     = new PtoSS(hwm, telemetry, "pto",
                "frontLeft", "frontRight", "backLeft", "backRight");

        // Commands
        intakeCommands = new IntakeCommands(intake);
        liftCommands   = new LiftCommands(pto);

        // Safety: force PTO disengaged at startup before any loop runs
        pto.disengagePtoCMD();
        pto.write();
    }

    // ── Robot start ───────────────────────────────────────────────────────────
    public void start() {
        follower.setStartingPose(new Pose(0, 0, isRedAlliance ? HEADING_RED : HEADING_BLUE));
        follower.startTeleopDrive();
    }

    // ── Main teleop loop ──────────────────────────────────────────────────────

    /**
     * Call this every loop iteration in your TeleOp OpMode.
     * Handles all driving, subsystem updates, and command logic.
     *
     * @param gamepad1 driver gamepad
     */
    public void update(Gamepad gamepad1) {
        // ── Driving ───────────────────────────────────────────────────────────
        double headingOffset = isRedAlliance ? HEADING_RED : HEADING_BLUE;

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

        // ── Button edge detection ─────────────────────────────────────────────
        boolean intakePressed    = gamepad1.a            && !lastIntake;
        boolean outtakePressed   = gamepad1.b            && !lastOuttake;
        boolean transferPressed  = gamepad1.x            && !lastTransfer;
        boolean liftPressed      = gamepad1.y            && !lastLift;
        boolean disengagePressed = gamepad1.right_bumper && !lastDisengage;

        // ── Intake commands ───────────────────────────────────────────────────
        if (intakePressed) {
            if (intakeCommands.isIntaking()) intakeCommands.idle();
            else                            intakeCommands.intake();
        }
        if (outtakePressed) {
            if (intakeCommands.isOuttaking()) intakeCommands.idle();
            else                             intakeCommands.outtake();
        }
        if (transferPressed) {
            intakeCommands.transfer();
        }

        // ── Lift commands ─────────────────────────────────────────────────────
        if (liftPressed)      liftCommands.lift();
        if (disengagePressed) liftCommands.disengage();

        // ── Save button states for next frame ─────────────────────────────────
        lastIntake    = gamepad1.a;
        lastOuttake   = gamepad1.b;
        lastTransfer  = gamepad1.x;
        lastLift      = gamepad1.y;
        lastDisengage = gamepad1.right_bumper;

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
    public Follower getFollower()    { return follower;       }
    public boolean  isRedAlliance()  { return isRedAlliance;  }
}