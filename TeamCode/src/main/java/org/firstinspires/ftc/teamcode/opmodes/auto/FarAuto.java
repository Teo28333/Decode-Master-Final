package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RobotAuton;

abstract class FarAuto extends OpMode {
    private static final double FIELD_SIZE = 144.0;
    private static final double PRELOAD_SPIN_UP_MS = 2250.0;
    private static final double PRELOAD_SHOOT_MS = 1750.0;
    private static final double SHOOT_MS = 1750.0;

    private RobotAuton robot;
    private PathChain[] paths;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private AutoStep autoStep = AutoStep.PRELOAD_SPIN_UP;

    private enum AutoStep {
        PRELOAD_SPIN_UP,
        PRELOAD_SHOOT,
        PRELOAD_FEED,
        LINE_INTAKE,
        LINE_RETURN,
        LINE_SHOOT,
        LINE_FEED,
        ZONE_INTAKE,
        ZONE_CONFIRM_INTAKE,
        ZONE_RETURN,
        ZONE_RETURN_TO_SHOOT,
        ZONE_SHOOT,
        ZONE_FEED,
        OVERFLOW_INTAKE,
        OVERFLOW_RETURN,
        OVERFLOW_SHOOT,
        OVERFLOW_FEED,
        DONE
    }

    protected abstract boolean isBlueAlliance();

    @Override
    public void init() {
        robot = new RobotAuton(hardwareMap, telemetry, isBlueAlliance());
        robot.disengagePto();
        Pose startPose = buildPaths();
        robot.start(startPose);

        telemetry.addLine(opModeName() + " ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addLine(opModeName() + " ready");
        telemetry.addData("Step", autoStep);
        telemetry.update();
    }

    @Override
    public void start() {
        autoStep = AutoStep.PRELOAD_SPIN_UP;
        stepTimer.reset();
    }

    @Override
    public void loop() {
        robot.update();

        if (autoStep == AutoStep.PRELOAD_SPIN_UP) {
            if (stepTimer.milliseconds() >= PRELOAD_SPIN_UP_MS && robot.isShooterReady()) {
                autoStep = AutoStep.PRELOAD_SHOOT;
            }
        } else if (isWaitingForShooter()) {
            startFeedWhenShooterReady();
        } else if (!robot.isBusy()) {
            startNextStep();
        }

        Pose pose = robot.getFollower().getPose();
        telemetry.addData("Auto", opModeName());
        telemetry.addData("Step", autoStep);
        telemetry.addData("Pose", "%.1f, %.1f, %.1f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }

    private Pose buildPaths() {
        Pose startingPose = new Pose(56.000, 8.500, Math.toRadians(180));
        Pose shootingPosition = new Pose(56.000, 10.000);
        Pose lineIntakeControl = new Pose(51.000, 39.000);
        Pose lineIntakePose = new Pose(10.000, 31.000);
        Pose controlZoneIntake = new Pose(16,10);
        Pose zoneIntakePose = new Pose(10.000, 10.000);
        Pose overflowIntakeControl = new Pose(8.000, 10.000);
        Pose overflowIntakePose = new Pose(10.000, 53.000);

        Pose shootingPose = withHeading(shootingPosition, headingTo(shootingPosition, lineIntakeControl));

        paths = new PathChain[] {
                tangentCurve(shootingPose, lineIntakeControl, lineIntakePose, false),
                tangentLine(lineIntakePose, shootingPose, true),
                tangentLine(shootingPose, zoneIntakePose, false),
                tangentLine(zoneIntakePose, controlZoneIntake, true),
                tangentLine(controlZoneIntake, zoneIntakePose,false),
                tangentLine(zoneIntakePose, shootingPose, true),
                tangentCurve(shootingPose, overflowIntakeControl, overflowIntakePose, false),
                tangentCurve(overflowIntakePose, overflowIntakeControl, shootingPose, true)
        };

        return alliancePose(startingPose);
    }

    private void startNextStep() {
        switch (autoStep) {
            case PRELOAD_SHOOT:
                autoStep = AutoStep.PRELOAD_FEED;
                robot.transferFor(PRELOAD_SHOOT_MS);
                break;

            case PRELOAD_FEED:
                autoStep = AutoStep.LINE_INTAKE;
                robot.followPathAndIntake(paths[0]);
                break;

            case LINE_INTAKE:
                autoStep = AutoStep.LINE_RETURN;
                robot.followPath(paths[1]);
                break;

            case LINE_RETURN:
                autoStep = AutoStep.LINE_SHOOT;
                break;

            case LINE_SHOOT:
                autoStep = AutoStep.LINE_FEED;
                robot.transferFor(SHOOT_MS);
                break;

            case LINE_FEED:
                autoStep = AutoStep.ZONE_INTAKE;
                robot.followPathAndIntake(paths[2]);
                break;

            case ZONE_INTAKE:
                autoStep = AutoStep.ZONE_CONFIRM_INTAKE;
                robot.followPathAndIntake(paths[3]);
                break;

            case ZONE_CONFIRM_INTAKE:
                autoStep = AutoStep.ZONE_RETURN;
                robot.followPathAndIntake(paths[4]);
                break;

            case ZONE_RETURN:
                autoStep = AutoStep.ZONE_RETURN_TO_SHOOT;
                robot.followPath(paths[5]);
                break;

            case ZONE_RETURN_TO_SHOOT:
                autoStep = AutoStep.ZONE_SHOOT;
                break;

            case ZONE_SHOOT:
                autoStep = AutoStep.ZONE_FEED;
                robot.transferFor(SHOOT_MS);
                break;

            case ZONE_FEED:
                autoStep = AutoStep.OVERFLOW_INTAKE;
                robot.followPathAndIntake(paths[6]);
                break;

            case OVERFLOW_INTAKE:
                autoStep = AutoStep.OVERFLOW_RETURN;
                robot.followPath(paths[7]);
                break;

            case OVERFLOW_RETURN:
                autoStep = AutoStep.OVERFLOW_SHOOT;
                break;

            case OVERFLOW_SHOOT:
                autoStep = AutoStep.OVERFLOW_FEED;
                robot.transferFor(SHOOT_MS);
                break;

            case OVERFLOW_FEED:
                autoStep = AutoStep.DONE;
                break;

            case DONE:
                autoStep = AutoStep.DONE;
                break;

            default:
                autoStep = AutoStep.DONE;
                break;
        }
    }

    private boolean isWaitingForShooter() {
        return autoStep == AutoStep.PRELOAD_SHOOT
                || autoStep == AutoStep.LINE_SHOOT
                || autoStep == AutoStep.ZONE_SHOOT
                || autoStep == AutoStep.OVERFLOW_SHOOT;
    }

    private void startFeedWhenShooterReady() {
        if (robot.isShooterReady()) {
            startNextStep();
        }
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

    private Pose withHeading(Pose pose, double heading) {
        return new Pose(pose.getX(), pose.getY(), heading);
    }

    private double headingTo(Pose start, Pose end) {
        return Math.atan2(end.getY() - start.getY(), end.getX() - start.getX());
    }

    private String opModeName() {
        return isBlueAlliance() ? "PP Move Only Blue" : "PP Move Only Red";
    }
}
