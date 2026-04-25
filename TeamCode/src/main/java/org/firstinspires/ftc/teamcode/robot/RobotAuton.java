package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private final PtoSS    pto; // private — no lift access in auton

    // ── Commands ──────────────────────────────────────────────────────────────
    public final IntakeCommands intakeCommands;
    // LiftCommands intentionally absent — lift is locked out in auton

    // ── Alliance ──────────────────────────────────────────────────────────────
    private final boolean isRedAlliance;

    private static final double HEADING_RED  = 0.0;
    private static final double HEADING_BLUE = Math.PI;

    // ── Goal position ─────────────────────────────────────────────────────────
    private static final double GOAL_X_RED  = 12.0;
    private static final double GOAL_Y_RED  = 120.0;
    private static final double GOAL_X_BLUE = 132.0;
    private static final double GOAL_Y_BLUE = 120.0;

    public final double goalX;
    public final double goalY;

    // ── Constructor ───────────────────────────────────────────────────────────
    public RobotAuton(HardwareMap hwm, Telemetry telemetry, boolean isRedAlliance) {
        this.isRedAlliance = isRedAlliance;

        goalX = isRedAlliance ? GOAL_X_RED  : GOAL_X_BLUE;
        goalY = isRedAlliance ? GOAL_Y_RED  : GOAL_Y_BLUE;

        follower = Constants.createFollower(hwm);

        intake  = new IntakeSS(hwm, telemetry, "intake1", "intake2", "gate", "led1");
        shooter = new ShooterSS(hwm, telemetry, "shooter1", "shooter2", "hood", "led2");
        turret  = new TurretSS(hwm, telemetry, "turret1", "turret2");
        pto     = new PtoSS(hwm, telemetry, "pto",
                "frontLeft", "frontRight", "backLeft", "backRight");

        intakeCommands = new IntakeCommands(intake);

        // Force PTO disengaged immediately at startup
        pto.disengagePtoCMD();
        pto.write();
    }

    // ── Robot start ───────────────────────────────────────────────────────────
    public void start(Pose startingPose) {
        follower.setStartingPose(startingPose);
    }

    // ── Main auton loop ───────────────────────────────────────────────────────
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
        turret.aimAtTargetCMD(robotX, robotY, heading, robotVel, goalX, goalY);
        shooter.shooterSpinCMD(robotX, robotY, heading, robotVel, goalX, goalY);

        // ── Command updates ───────────────────────────────────────────────────
        intakeCommands.update(
                robotX, robotY,
                shooter.isReady(),
                !turret.robotNeedToTurn()
        );

        // ── Subsystem updates ─────────────────────────────────────────────────
        intake.update();
        shooter.update();
        turret.update();
        pto.update(); // pushes disengagedPos set above
    }

    // ── Getters ───────────────────────────────────────────────────────────────
    public Follower getFollower()   { return follower;      }
    public boolean  isRedAlliance() { return isRedAlliance; }
}