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
    private static final double PRELOAD_SHOOT_MS = 1000.0;
    private static final double SHOOT_MS = 1000.0;
    private static final double HUMAN_PLAYER_CONFIRM_INTAKE_MS = 1000.0;
    private static final double RETURN_INTAKE_MS = 250.0;

    private RobotAuton robot;
    private PathChain[] paths;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private AutoStep autoStep = AutoStep.PRELOAD_SPIN_UP;

    private enum AutoStep {
        PRELOAD_SPIN_UP,
        PRELOAD_SHOOT,
        PRELOAD_FEED,
        SPIKE_INTAKE,
        SPIKE_RETURN,
        SPIKE_SHOOT,
        SPIKE_FEED,
        HUMAN_PLAYER_INTAKE,
        HUMAN_PLAYER_CONFIRM_INTAKE,
        HUMAN_PLAYER_RETURN,
        HUMAN_PLAYER_SHOOT,
        HUMAN_PLAYER_FEED,
        FLOW_1_INTAKE,
        FLOW_1_RETURN,
        FLOW_1_SHOOT,
        FLOW_1_FEED,
        FLOW_2_INTAKE,
        FLOW_2_RETURN,
        FLOW_2_SHOOT,
        FLOW_2_FEED,
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

    @Override
    public void stop() {
        robot.savePose();
    }

    private Pose buildPaths() {
        Pose startingPose = new Pose(56.000, 8.500, Math.toRadians(180));

        paths = new PathChain[] {
                tangentCurve(
                        startingPose,
                        new Pose(54.213, 28.075),
                        new Pose(32.267, 38.760),
                        new Pose(15.000, 36.000),
                        false
                ),
                tangentLine(new Pose(15.000, 36.000), new Pose(51.863, 13.768), true),
                tangentLine(new Pose(51.863, 13.768), new Pose(10.000, 9.500), false),
                tangentLine(new Pose(10.000, 9.500), new Pose(48.000, 9.000), true),
                tangentCurve(
                        new Pose(48.000, 9.000),
                        new Pose(44.076, 31.476),
                        new Pose(34.878, 44.499),
                        new Pose(10.000, 44.253),
                        false
                ),
                tangentLine(new Pose(10.000, 44.253), new Pose(53.000, 17.000), true),
                tangentLine(new Pose(53.000, 17.000), new Pose(10.000, 10.000), false),
                tangentLine(new Pose(10.000, 10.000), new Pose(48.000, 10.000), true)
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
                autoStep = AutoStep.SPIKE_INTAKE;
                robot.followPathAndIntake(paths[0]);
                break;

            case SPIKE_INTAKE:
                autoStep = AutoStep.SPIKE_RETURN;
                robot.followPathAndIntakeFor(paths[1], RETURN_INTAKE_MS);
                break;

            case SPIKE_RETURN:
                autoStep = AutoStep.SPIKE_SHOOT;
                break;

            case SPIKE_SHOOT:
                autoStep = AutoStep.SPIKE_FEED;
                robot.transferFor(SHOOT_MS);
                break;

            case SPIKE_FEED:
                autoStep = AutoStep.HUMAN_PLAYER_INTAKE;
                robot.followPathAndIntake(paths[2]);
                break;

            case HUMAN_PLAYER_INTAKE:
                autoStep = AutoStep.HUMAN_PLAYER_CONFIRM_INTAKE;
                robot.intakeFor(HUMAN_PLAYER_CONFIRM_INTAKE_MS);
                break;

            case HUMAN_PLAYER_CONFIRM_INTAKE:
                autoStep = AutoStep.HUMAN_PLAYER_RETURN;
                robot.followPathAndIntakeFor(paths[3], RETURN_INTAKE_MS);
                break;

            case HUMAN_PLAYER_RETURN:
                autoStep = AutoStep.HUMAN_PLAYER_SHOOT;
                break;

            case HUMAN_PLAYER_SHOOT:
                autoStep = AutoStep.HUMAN_PLAYER_FEED;
                robot.transferFor(SHOOT_MS);
                break;

            case HUMAN_PLAYER_FEED:
                autoStep = AutoStep.FLOW_1_INTAKE;
                robot.followPathAndIntake(paths[4]);
                break;

            case FLOW_1_INTAKE:
                autoStep = AutoStep.FLOW_1_RETURN;
                robot.followPathAndIntakeFor(paths[5], RETURN_INTAKE_MS);
                break;

            case FLOW_1_RETURN:
                autoStep = AutoStep.FLOW_1_SHOOT;
                break;

            case FLOW_1_SHOOT:
                autoStep = AutoStep.FLOW_1_FEED;
                robot.transferFor(SHOOT_MS);
                break;

            case FLOW_1_FEED:
                autoStep = AutoStep.FLOW_2_INTAKE;
                robot.followPathAndIntake(paths[6]);
                break;

            case FLOW_2_INTAKE:
                autoStep = AutoStep.FLOW_2_RETURN;
                robot.followPathAndIntakeFor(paths[7], RETURN_INTAKE_MS);
                break;

            case FLOW_2_RETURN:
                autoStep = AutoStep.FLOW_2_SHOOT;
                break;

            case FLOW_2_SHOOT:
                autoStep = AutoStep.FLOW_2_FEED;
                robot.transferFor(SHOOT_MS);
                break;

            case FLOW_2_FEED:
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
                || autoStep == AutoStep.SPIKE_SHOOT
                || autoStep == AutoStep.HUMAN_PLAYER_SHOOT
                || autoStep == AutoStep.FLOW_1_SHOOT
                || autoStep == AutoStep.FLOW_2_SHOOT;
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

    private PathChain tangentCurve(Pose blueStart, Pose blueControl1, Pose blueControl2,
                                   Pose blueEnd, boolean reversed) {
        Pose start = alliancePose(blueStart);
        Pose control1 = alliancePose(blueControl1);
        Pose control2 = alliancePose(blueControl2);
        Pose end = alliancePose(blueEnd);
        PathBuilder builder = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(start, control1, control2, end))
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
