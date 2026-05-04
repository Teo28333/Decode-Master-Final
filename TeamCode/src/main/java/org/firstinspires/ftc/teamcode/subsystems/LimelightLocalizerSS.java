package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.robot.RobotConstants.*;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;

public class LimelightLocalizerSS {
    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private final ElapsedTime relocalizeTimer = new ElapsedTime();

    private boolean available = false;
    private boolean instantReset = false;
    private boolean lastRelocalizeSucceeded = false;
    private String lastStatus = "not initialized";
    private Pose lastCurrentPose = null;
    private Pose lastVisionPose = null;
    private double lastCorrectionIn = 0.0;
    private double lastHeadingCorrectionDeg = 0.0;
    private double lastVelocityInPerSec = 0.0;
    private double lastAngularVelocityDegPerSec = 0.0;
    private double lastStalenessMs = 0.0;
    private double lastPositionStddev = 0.0;
    private double lastHeadingStddevDeg = 0.0;
    private int lastTagCount = 0;

    public LimelightLocalizerSS(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.tryGet(Limelight3A.class, LIMELIGHT_NAME);

        if (limelight == null) {
            lastStatus = "missing hardware: " + LIMELIGHT_NAME;
            return;
        }

        available = true;
        limelight.setPollRateHz(LIMELIGHT_POLL_RATE_HZ);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();
        relocalizeTimer.reset();
        lastStatus = "started";
    }

    public void setInstantReset(boolean instantReset) {
        this.instantReset = instantReset;
    }

    public void startLimelight(int pollRate, int pipeline) {
        if (!available || limelight == null) {
            lastStatus = "not available";
            return;
        }

        limelight.setPollRateHz(pollRate);
        limelight.pipelineSwitch(pipeline);
        limelight.start();
        relocalizeTimer.reset();
        lastStatus = "started";
    }

    public boolean relocalizeIfDue(Follower follower) {
        if (relocalizeTimer.milliseconds() < LIMELIGHT_RELOCALIZE_INTERVAL_MS) {
            return false;
        }

        updateFollowerPoseFromLimelight(follower);
        return lastRelocalizeSucceeded;
    }

    public boolean relocalizeNow(Follower follower) {
        updateFollowerPoseFromLimelight(follower);
        return true;
    }

    public void updateFollowerPoseFromLimelight(Follower follower) {
        lastRelocalizeSucceeded = false;

        if (!LIMELIGHT_RELOCALIZATION_ENABLED) {
            lastStatus = "disabled";
            return;
        }
        if (!available || limelight == null || follower == null) {
            lastStatus = "not available";
            return;
        }
        if (follower.getPose() == null) {
            lastStatus = "no follower pose";
            return;
        }

        lastCurrentPose = follower.getPose();
        lastVelocityInPerSec = follower.getVelocity() == null
                ? 0.0
                : Math.hypot(
                        follower.getVelocity().getXComponent(),
                        follower.getVelocity().getYComponent()
                );
        lastAngularVelocityDegPerSec = Math.toDegrees(Math.abs(follower.getAngularVelocity()));

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            lastStatus = "invalid result";
            return;
        }
        lastStalenessMs = result.getStaleness();
        lastTagCount = result.getBotposeTagCount();

        Pose3D botpose = result.getBotpose();
        Pose3D botposeMT2 = result.getBotpose_MT2();
        if (botpose == null) {
            lastStatus = "missing botpose";
            return;
        }

        double rawX = botpose.getPosition().x;
        double rawY = botpose.getPosition().y;
        double inchX = rawY / DistanceUnit.mPerInch;
        double inchY = -(rawX) / DistanceUnit.mPerInch;
        double heading = botpose.getOrientation().getYaw(AngleUnit.DEGREES) - 90;

        double pedroX = inchX + 72;
        double pedroY = inchY + 72;
        Pose pedroPose = new Pose(pedroX, pedroY, Math.toRadians(heading));
        lastVisionPose = pedroPose;

        lastCorrectionIn = Math.hypot(
                pedroPose.getX() - lastCurrentPose.getX(),
                pedroPose.getY() - lastCurrentPose.getY()
        );
        lastHeadingCorrectionDeg = Math.toDegrees(Math.abs(normalizeAngle(
                pedroPose.getHeading() - lastCurrentPose.getHeading()
        )));

        double[] stddev = LIMELIGHT_USE_MEGATAG2 ? result.getStddevMt2() : result.getStddevMt1();
        lastPositionStddev = 0.0;
        lastHeadingStddevDeg = 0.0;
        if (stddev != null && stddev.length >= 6) {
            lastPositionStddev = Math.max(Math.abs(stddev[0]), Math.abs(stddev[1]));
            lastHeadingStddevDeg = Math.abs(stddev[5]);
        }

        if (!PoseStorage.isValid(pedroPose)) {
            relocalizeTimer.reset();
            lastStatus = "rejected: outside field";
            return;
        }
        if (lastStalenessMs > LIMELIGHT_MAX_STALENESS_MS) {
            relocalizeTimer.reset();
            lastStatus = "rejected: stale";
            return;
        }
        if (lastPositionStddev > LIMELIGHT_MAX_POSITION_STDDEV_IN) {
            relocalizeTimer.reset();
            lastStatus = "rejected: position stddev";
            return;
        }
        if (lastHeadingStddevDeg > LIMELIGHT_MAX_HEADING_STDDEV_DEG) {
            relocalizeTimer.reset();
            lastStatus = "rejected: heading stddev";
            return;
        }
        if (lastVelocityInPerSec > LIMELIGHT_MAX_RELOCALIZE_SPEED_IN_PER_SEC) {
            relocalizeTimer.reset();
            lastStatus = "rejected: moving";
            return;
        }
        if (lastAngularVelocityDegPerSec > LIMELIGHT_MAX_RELOCALIZE_TURN_DEG_PER_SEC) {
            relocalizeTimer.reset();
            lastStatus = "rejected: turning";
            return;
        }
        if (!instantReset && lastCorrectionIn > LIMELIGHT_MAX_CORRECTION_IN) {
            relocalizeTimer.reset();
            lastStatus = "rejected: correction jump";
            return;
        }

        if (instantReset) {
            instantReset = false;
            if (LIMELIGHT_APPLY_POSE_UPDATES) {
                follower.setPose(pedroPose);
                PoseStorage.setCurrentPose(pedroPose);
            }
            relocalizeTimer.reset();
            lastRelocalizeSucceeded = true;
            lastStatus = LIMELIGHT_APPLY_POSE_UPDATES
                    ? "instant reset"
                    : "accepted: instant telemetry only";
            return;
        }

        /*
        if (relocalizeTimer.seconds() < RELOCALIZATION_INTERVAL) return;

        double velocity = follower.getVelocity().getMagnitude();
        if (velocity > MAX_VELOCITY) return;

        Pose current = follower.getPose();
        double driftX = Math.abs(current.getX() - inchX);
        double driftY = Math.abs(current.getY() - inchY);

        if (driftX < MAX_DRIFT && driftY < MAX_DRIFT) {
            follower.setPose(pedroPose);
        }
        */

        if (LIMELIGHT_APPLY_POSE_UPDATES) {
            follower.setPose(pedroPose);
            PoseStorage.setCurrentPose(pedroPose);
        }
        relocalizeTimer.reset();
        lastRelocalizeSucceeded = true;
        lastStatus = LIMELIGHT_APPLY_POSE_UPDATES
                ? "accepted: pose applied"
                : "accepted: telemetry only";
    }

    public void updateRobotOrientation(Pose pose) {
        if (available && limelight != null && pose != null) {
            limelight.updateRobotOrientation(Math.toDegrees(pose.getHeading()));
        }
    }

    public void telemetry() {
        telemetry.addData("Limelight available", available);
        telemetry.addData("Limelight relocalized", lastRelocalizeSucceeded);
        telemetry.addData("Limelight status", lastStatus);
        telemetry.addData("Limelight next relocalize ms",
                Math.max(0.0, LIMELIGHT_RELOCALIZE_INTERVAL_MS - relocalizeTimer.milliseconds()));
        if (lastVisionPose != null) {
            telemetry.addData("Limelight pose", "%.1f, %.1f, %.1f",
                    lastVisionPose.getX(),
                    lastVisionPose.getY(),
                    Math.toDegrees(lastVisionPose.getHeading()));
        }
        if (lastCurrentPose != null && lastVisionPose != null) {
            telemetry.addData("Limelight correction in", "%.1f", lastCorrectionIn);
            telemetry.addData("Limelight heading correction deg", "%.1f", lastHeadingCorrectionDeg);
        }
        telemetry.addData("Limelight tag count", lastTagCount);
        telemetry.addData("Limelight staleness ms", "%.0f", lastStalenessMs);
        telemetry.addData("Limelight pos stddev", "%.1f", lastPositionStddev);
        telemetry.addData("Limelight heading stddev deg", "%.1f", lastHeadingStddevDeg);
        telemetry.addData("Limelight robot speed ips", "%.1f", lastVelocityInPerSec);
        telemetry.addData("Limelight robot turn dps", "%.1f", lastAngularVelocityDegPerSec);
    }

    public Pose getLastVisionPose() {
        return lastVisionPose;
    }

    private static double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }

}
