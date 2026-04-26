package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.RobotAuton;

abstract class FarAuto extends OpMode {
    private static final double FIELD_SIZE = 144.0;

    private RobotAuton robot;
    private Pose startPose;
    private PathChain[] paths;
    private int pathIndex;

    protected abstract boolean isBlueAlliance();

    @Override
    public void init() {
        robot = new RobotAuton(hardwareMap, telemetry, isBlueAlliance());
        buildPaths();
        robot.start(startPose);

        telemetry.addLine(opModeName() + " ready");
        telemetry.update();
    }

    @Override
    public void start() {
        pathIndex = 0;
        followNextPath();
    }

    @Override
    public void loop() {
        robot.update();

        if (!robot.getFollower().isBusy()) {
            followNextPath();
        }

        Pose pose = robot.getFollower().getPose();
        telemetry.addData("Auto", opModeName());
        telemetry.addData("Path", "%d/%d", Math.min(pathIndex, paths.length), paths.length);
        telemetry.addData("Pose", "%.1f, %.1f, %.1f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }

    private void buildPaths() {
        Pose shotPose = new Pose(56.000, 10.000);
        Pose middleControl = new Pose(51.000, 39.000);
        Pose middlePose = new Pose(11.000, 37.000);
        Pose lowPose = new Pose(10.000, 11.000);
        Pose highControl = new Pose(8.000, 17.000);
        Pose highPose = new Pose(10.000, 53.000);

        startPose = alliancePose(shotPose);
        paths = new PathChain[] {
                tangentCurve(shotPose, middleControl, middlePose, false),
                tangentLine(middlePose, shotPose, true),
                tangentLine(shotPose, lowPose, false),
                tangentLine(lowPose, shotPose, true),
                tangentCurve(shotPose, highControl, highPose, false),
                tangentCurve(highPose, highControl, shotPose, false)
        };
    }

    private void followNextPath() {
        if (pathIndex >= paths.length) {
            return;
        }

        robot.getFollower().followPath(paths[pathIndex], 1.0, true);
        pathIndex++;
    }

    private PathChain tangentLine(Pose blueStart, Pose blueEnd, boolean reversed) {
        Pose start = alliancePose(blueStart);
        Pose end = alliancePose(blueEnd);
        PathBuilder builder = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(start, end))
                .setTangentHeadingInterpolation();

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    private PathChain tangentCurve(Pose blueStart, Pose blueControl, Pose blueEnd, boolean reversed) {
        Pose start = alliancePose(blueStart);
        Pose control = alliancePose(blueControl);
        Pose end = alliancePose(blueEnd);
        PathBuilder builder = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setTangentHeadingInterpolation();

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    private Pose alliancePose(Pose bluePose) {
        if (isBlueAlliance()) {
            return bluePose;
        }

        return new Pose(FIELD_SIZE - bluePose.getX(), bluePose.getY(), Math.PI - bluePose.getHeading());
    }

    private String opModeName() {
        return isBlueAlliance() ? "PP Move Only Blue" : "PP Move Only Red";
    }
}
