package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.LimelightLocalizerSS;

@TeleOp(name = "Limelight Pose Tuning", group = "Tuning")
public class LimelightPoseTuning extends OpMode {
    private Follower follower;
    private LimelightLocalizerSS limelightLocalizer;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void init() {
        RobotConstants.LIMELIGHT_APPLY_POSE_UPDATES = false;

        follower = Constants.createFollower(hardwareMap);
        limelightLocalizer = new LimelightLocalizerSS(hardwareMap, telemetry);
        follower.setStartingPose(PoseStorage.hasValidPose()
                ? PoseStorage.currentPose
                : PoseStorage.fieldCenterPose());
        follower.setMaxPower(RobotConstants.DRIVE_SPEED_MULTIPLIER);
        follower.startTeleopDrive();

        telemetry.addLine("Limelight Pose Tuning ready");
        telemetry.addLine("Drive with sticks. D-pad up = center pose, D-pad down = saved/start pose.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        boolean resetCenterPressed = gamepad1.dpad_up && !lastDpadUp;
        boolean resetStartPressed = gamepad1.dpad_down && !lastDpadDown;

        if (resetCenterPressed) {
            resetPose(PoseStorage.fieldCenterPose());
        } else if (resetStartPressed) {
            resetPose(PoseStorage.allianceStartPose(true));
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * RobotConstants.DRIVE_SPEED_MULTIPLIER,
                -gamepad1.left_stick_x * RobotConstants.DRIVE_SPEED_MULTIPLIER,
                -gamepad1.right_stick_x * 0.75 * RobotConstants.DRIVE_SPEED_MULTIPLIER,
                false,
                Math.toRadians(180)
        );
        follower.update();

        limelightLocalizer.updateRobotOrientation(follower.getPose());
        limelightLocalizer.relocalizeNow(follower);
        PoseStorage.setCurrentPose(follower.getPose());

        Pose pose = follower.getPose();
        telemetry.addLine("Limelight Pose Tuning");
        telemetry.addData("Pedro pose", "%.1f, %.1f, %.1f",
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading()));
        telemetry.addData("Pedro velocity", "%.1f, %.1f",
                follower.getVelocity() == null ? 0.0 : follower.getVelocity().getXComponent(),
                follower.getVelocity() == null ? 0.0 : follower.getVelocity().getYComponent());
        telemetry.addData("Pedro angular vel deg/s", "%.1f", Math.toDegrees(follower.getAngularVelocity()));
        limelightLocalizer.telemetry();
        telemetry.update();

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
    }

    @Override
    public void stop() {
        PoseStorage.setCurrentPose(follower.getPose());
    }

    private void resetPose(Pose pose) {
        if (PoseStorage.isValid(pose)) {
            follower.setPose(pose);
            PoseStorage.setCurrentPose(pose);
        }
    }
}
