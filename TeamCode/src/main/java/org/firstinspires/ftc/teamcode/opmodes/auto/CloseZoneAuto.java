package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RobotAuton;

abstract class CloseZoneAuto extends OpMode {
    private static final double FIELD_SIZE = 144.0;
    private static final double PRELOAD_SHOOT_MS = 1000.0;
    private static final double SHOOT_MS = 1000.0;
    private static final double GATE_CONFIRM_INTAKE_MS = 1000.0;
    private static final double RETURN_INTAKE_MS = 250.0;

    private RobotAuton robot;
    private Paths paths;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private AutoStep autoStep = AutoStep.GO_TO_SHOOTING_POSE;

    private enum AutoStep {
        GO_TO_SHOOTING_POSE,
        PRELOAD_SHOOT,
        PRELOAD_FEED,
        MIDDLE_INTAKE,
        MIDDLE_RETURN,
        MIDDLE_SHOOT,
        MIDDLE_FEED,
        GATE_INTAKE,
        GATE_CONFIRM_INTAKE,
        GATE_RETURN,
        GATE_SHOOT,
        GATE_FEED,
        GATE_2_INTAKE,
        GATE_2_CONFIRM_INTAKE,
        GATE_2_RETURN,
        GATE_2_SHOOT,
        GATE_2_FEED,
        GATE_3_INTAKE,
        GATE_3_CONFIRM_INTAKE,
        GATE_3_RETURN,
        GATE_3_SHOOT,
        GATE_3_FEED,
        GATE_4_INTAKE,
        GATE_4_CONFIRM_INTAKE,
        GATE_4_RETURN,
        GATE_4_SHOOT,
        GATE_4_FEED,
        RIGHT_INTAKE,
        RIGHT_RETURN,
        RIGHT_SHOOT,
        RIGHT_FEED,
        LEAVING_ZONE,
        DONE
    }

    protected abstract boolean isBlueAlliance();

    @Override
    public void init() {
        robot = new RobotAuton(hardwareMap, telemetry, isBlueAlliance());
        robot.disengagePto();
        paths = new Paths();
        robot.start(alliancePose(new Pose(18.603, 120.278, Math.toRadians(-40))));

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
        autoStep = AutoStep.GO_TO_SHOOTING_POSE;
        robot.followPath(paths.goToShootingPose);
        stepTimer.reset();
    }

    @Override
    public void loop() {
        robot.update();

        if (isWaitingForShooter()) {
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

    private void startNextStep() {
        switch (autoStep) {
            case GO_TO_SHOOTING_POSE:
                autoStep = AutoStep.PRELOAD_SHOOT;
                break;

            case PRELOAD_SHOOT:
                autoStep = AutoStep.PRELOAD_FEED;
                robot.transferFor(PRELOAD_SHOOT_MS);
                break;

            case PRELOAD_FEED:
                autoStep = AutoStep.MIDDLE_INTAKE;
                robot.followPathAndIntake(paths.intakeMiddleLine);
                break;

            case MIDDLE_INTAKE:
                autoStep = AutoStep.MIDDLE_RETURN;
                robot.followPathAndIntakeFor(paths.goToShootingPose2, RETURN_INTAKE_MS);
                break;

            case MIDDLE_RETURN:
                autoStep = AutoStep.MIDDLE_SHOOT;
                break;

            case MIDDLE_SHOOT:
                autoStep = AutoStep.MIDDLE_FEED;
                robot.transferFor(SHOOT_MS);
                break;

            case MIDDLE_FEED:
                autoStep = AutoStep.GATE_INTAKE;
                robot.followPathAndIntake(paths.gateIntake1);
                break;

            case GATE_INTAKE:
                autoStep = AutoStep.GATE_CONFIRM_INTAKE;
                robot.intakeFor(GATE_CONFIRM_INTAKE_MS);
                break;

            case GATE_CONFIRM_INTAKE:
                autoStep = AutoStep.GATE_RETURN;
                robot.followPathAndIntakeFor(paths.goToShootingPose3, RETURN_INTAKE_MS);
                break;

            case GATE_RETURN:
                autoStep = AutoStep.GATE_SHOOT;
                break;

            case GATE_SHOOT:
                autoStep = AutoStep.GATE_FEED;
                robot.transferFor(SHOOT_MS);
                break;

            case GATE_FEED:
                autoStep = AutoStep.GATE_2_INTAKE;
                robot.followPathAndIntake(paths.gateIntake1);
                break;

            case GATE_2_INTAKE:
                autoStep = AutoStep.GATE_2_CONFIRM_INTAKE;
                robot.intakeFor(GATE_CONFIRM_INTAKE_MS);
                break;

            case GATE_2_CONFIRM_INTAKE:
                autoStep = AutoStep.GATE_2_RETURN;
                robot.followPathAndIntakeFor(paths.goToShootingPose3, RETURN_INTAKE_MS);
                break;

            case GATE_2_RETURN:
                autoStep = AutoStep.GATE_2_SHOOT;
                break;

            case GATE_2_SHOOT:
                autoStep = AutoStep.GATE_2_FEED;
                robot.transferFor(SHOOT_MS);
                break;

            case GATE_2_FEED:
                autoStep = AutoStep.GATE_3_INTAKE;
                robot.followPathAndIntake(paths.gateIntake1);
                break;

            case GATE_3_INTAKE:
                autoStep = AutoStep.GATE_3_CONFIRM_INTAKE;
                robot.intakeFor(GATE_CONFIRM_INTAKE_MS);
                break;

            case GATE_3_CONFIRM_INTAKE:
                autoStep = AutoStep.GATE_3_RETURN;
                robot.followPathAndIntakeFor(paths.goToShootingPose3, RETURN_INTAKE_MS);
                break;

            case GATE_3_RETURN:
                autoStep = AutoStep.GATE_3_SHOOT;
                break;

            case GATE_3_SHOOT:
                autoStep = AutoStep.GATE_3_FEED;
                robot.transferFor(SHOOT_MS);
                break;

            case GATE_3_FEED:
                autoStep = AutoStep.GATE_4_INTAKE;
                robot.followPathAndIntake(paths.gateIntake1);
                break;

            case GATE_4_INTAKE:
                autoStep = AutoStep.GATE_4_CONFIRM_INTAKE;
                robot.intakeFor(GATE_CONFIRM_INTAKE_MS);
                break;

            case GATE_4_CONFIRM_INTAKE:
                autoStep = AutoStep.GATE_4_RETURN;
                robot.followPathAndIntakeFor(paths.goToShootingPose3, RETURN_INTAKE_MS);
                break;

            case GATE_4_RETURN:
                autoStep = AutoStep.GATE_4_SHOOT;
                break;

            case GATE_4_SHOOT:
                autoStep = AutoStep.GATE_4_FEED;
                robot.transferFor(SHOOT_MS);
                break;

            case GATE_4_FEED:
                autoStep = AutoStep.RIGHT_INTAKE;
                robot.followPathAndIntake(paths.intakeRightLine);
                break;

            case RIGHT_INTAKE:
                autoStep = AutoStep.RIGHT_RETURN;
                robot.followPathAndIntakeFor(paths.goToShootingPose4, RETURN_INTAKE_MS);
                break;

            case RIGHT_RETURN:
                autoStep = AutoStep.RIGHT_SHOOT;
                break;

            case RIGHT_SHOOT:
                autoStep = AutoStep.RIGHT_FEED;
                robot.transferFor(SHOOT_MS);
                break;

            case RIGHT_FEED:
                autoStep = AutoStep.LEAVING_ZONE;
                robot.followPath(paths.leavingZone);
                break;

            case LEAVING_ZONE:
            case DONE:
            default:
                autoStep = AutoStep.DONE;
                break;
        }
    }

    private boolean isWaitingForShooter() {
        return autoStep == AutoStep.PRELOAD_SHOOT
                || autoStep == AutoStep.MIDDLE_SHOOT
                || autoStep == AutoStep.GATE_SHOOT
                || autoStep == AutoStep.GATE_2_SHOOT
                || autoStep == AutoStep.GATE_3_SHOOT
                || autoStep == AutoStep.GATE_4_SHOOT
                || autoStep == AutoStep.RIGHT_SHOOT;
    }

    private void startFeedWhenShooterReady() {
        if (robot.isShooterReady()) {
            startNextStep();
        }
    }

    private Pose alliancePose(Pose bluePose) {
        if (isBlueAlliance()) {
            return bluePose;
        }

        return new Pose(FIELD_SIZE - bluePose.getX(), bluePose.getY(), Math.PI - bluePose.getHeading());
    }

    private String opModeName() {
        return isBlueAlliance() ? "PP Close Zone Blue" : "PP Close Zone Red";
    }

    private class Paths {
        private final PathChain goToShootingPose;
        private final PathChain intakeMiddleLine;
        private final PathChain goToShootingPose2;
        private final PathChain gateIntake1;
        private final PathChain goToShootingPose3;
        private final PathChain intakeRightLine;
        private final PathChain goToShootingPose4;
        private final PathChain leavingZone;

        private Paths() {
            goToShootingPose = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    alliancePose(new Pose(18.603, 120.278, Math.toRadians(-40))),
                                    alliancePose(new Pose(42.478, 102.949)),
                                    alliancePose(new Pose(50.751, 81.026))
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            intakeMiddleLine = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    alliancePose(new Pose(50.751, 81.026)),
                                    alliancePose(new Pose(45.598, 62.729)),
                                    alliancePose(new Pose(11.874, 57.405))
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            goToShootingPose2 = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierLine(
                                    alliancePose(new Pose(11.874, 57.405)),
                                    alliancePose(new Pose(50.213, 81.026))
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            gateIntake1 = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    alliancePose(new Pose(50.213, 81.026)),
                                    alliancePose(new Pose(35.615, 48.126)),
                                    alliancePose(new Pose(11.469, 60.767))
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            goToShootingPose3 = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    alliancePose(new Pose(11.469, 60.767)),
                                    alliancePose(new Pose(20.194, 54.924)),
                                    alliancePose(new Pose(50.565, 81.026))
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeRightLine = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierLine(
                                    alliancePose(new Pose(50.565, 81.026)),
                                    alliancePose(new Pose(18.493, 83.650))
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            goToShootingPose4 = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierLine(
                                    alliancePose(new Pose(18.493, 83.650)),
                                    alliancePose(new Pose(31.409, 100.447))
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            leavingZone = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierLine(
                                    alliancePose(new Pose(31.409, 100.447)),
                                    alliancePose(new Pose(21.911, 71.509))
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}
