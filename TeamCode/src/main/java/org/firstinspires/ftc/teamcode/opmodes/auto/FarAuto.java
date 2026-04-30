package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.*;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.RobotAuton;

abstract class FarAuto extends OpMode {
    private RobotAuton robot;
    private PathChain[] paths;
    private final AutoStateMachine<AutoStep> autoState = new AutoStateMachine<>(AutoStep.PRELOAD_SPIN_UP);
    private boolean dryRun = false;
    private boolean lastX = false;

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
        Pose startPose = buildPaths();
        robot.start(startPose);

        telemetry.addLine(opModeName() + " ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        boolean xPressed = gamepad1.x && !lastX;
        if (xPressed) {
            dryRun = !dryRun;
        }
        lastX = gamepad1.x;

        telemetry.addLine(opModeName() + " ready");
        telemetry.addData("Step", autoState.getStep());
        telemetry.addData("Dry run", dryRun);
        telemetry.addData("Path timeout ms", PATH_STEP_TIMEOUT_MS);
        telemetry.addData("Shooter timeout ms", SHOOTER_WAIT_TIMEOUT_MS);
        telemetry.update();
    }

    @Override
    public void start() {
        autoState.setStep(AutoStep.PRELOAD_SPIN_UP);
    }

    @Override
    public void loop() {
        robot.update();

        if (autoState.getStep() == AutoStep.PRELOAD_SPIN_UP) {
            if ((dryRun && autoState.elapsedMs() >= FAR_PRELOAD_SPIN_UP_MS)
                    || (autoState.elapsedMs() >= FAR_PRELOAD_SPIN_UP_MS && robot.isShooterReady())) {
                setStep(AutoStep.PRELOAD_SHOOT);
            } else if (autoState.elapsedMs() >= FAR_PRELOAD_SPIN_UP_MS + SHOOTER_WAIT_TIMEOUT_MS) {
                setStep(AutoStep.PRELOAD_SHOOT);
            }
        } else if (isWaitingForShooter()) {
            startFeedWhenShooterReady();
        } else if (robot.isBusy() && autoState.elapsedMs() >= PATH_STEP_TIMEOUT_MS) {
            robot.forceIdle();
            startNextStep();
        } else if (dryRun && dryRunStepWaitMs() > 0.0) {
            if (autoState.elapsedMs() >= dryRunStepWaitMs()) {
                startNextStep();
            }
        } else if (!robot.isBusy()) {
            startNextStep();
        }

        Pose pose = robot.getFollower().getPose();
        telemetry.addData("Auto", opModeName());
        telemetry.addData("Step", autoState.getStep());
        telemetry.addData("Step time ms", "%.0f", autoState.elapsedMs());
        telemetry.addData("Path progress %%", "%.1f", robot.getPathProgressPercent());
        telemetry.addData("Dry run", dryRun);
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
                        new Pose(53.163, 28.075),
                        new Pose(34.716, 37.360),
                        new Pose(15.000, 36.000),
                        false
                ),
                tangentLine(new Pose(15.000, 36.000), new Pose(52.913, 17.267), true),
                tangentLine(new Pose(52.913, 17.267), new Pose(10.000, 10.050), false),
                tangentLine(new Pose(10.000, 10.050), new Pose(52.549, 13.899), true),
                tangentCurve(
                        new Pose(52.549, 13.899),
                        new Pose(44.076, 31.476),
                        new Pose(39.427, 39.250),
                        new Pose(10.000, 38.129),
                        false
                ),
                tangentLine(new Pose(10.000, 38.129), new Pose(53.000, 17.000), true),
                tangentLine(new Pose(53.000, 17.000), new Pose(10.000, 10.000), false),
                tangentLine(new Pose(10.000, 10.000), new Pose(48.350, 10.700), true)
        };

        return alliancePose(startingPose);
    }

    private void startNextStep() {
        switch (autoState.getStep()) {
            case PRELOAD_SHOOT:
                startTransfer(AutoStep.PRELOAD_FEED, FAR_PRELOAD_SHOOT_MS);
                break;

            case PRELOAD_FEED:
                setStep(AutoStep.SPIKE_INTAKE);
                followIntakePath(paths[0]);
                break;

            case SPIKE_INTAKE:
                setStep(AutoStep.SPIKE_RETURN);
                followReturnPath(paths[1]);
                break;

            case SPIKE_RETURN:
                setStep(AutoStep.SPIKE_SHOOT);
                break;

            case SPIKE_SHOOT:
                startTransfer(AutoStep.SPIKE_FEED, FAR_SHOOT_MS);
                break;

            case SPIKE_FEED:
                setStep(AutoStep.HUMAN_PLAYER_INTAKE);
                followIntakePath(paths[2]);
                break;

            case HUMAN_PLAYER_INTAKE:
                setStep(AutoStep.HUMAN_PLAYER_CONFIRM_INTAKE);
                intakeFor(FAR_HUMAN_PLAYER_CONFIRM_INTAKE_MS);
                break;

            case HUMAN_PLAYER_CONFIRM_INTAKE:
                setStep(AutoStep.HUMAN_PLAYER_RETURN);
                followReturnPath(paths[3]);
                break;

            case HUMAN_PLAYER_RETURN:
                setStep(AutoStep.HUMAN_PLAYER_SHOOT);
                break;

            case HUMAN_PLAYER_SHOOT:
                startTransfer(AutoStep.HUMAN_PLAYER_FEED, FAR_SHOOT_MS);
                break;

            case HUMAN_PLAYER_FEED:
                setStep(AutoStep.FLOW_1_INTAKE);
                followIntakePath(paths[4]);
                break;

            case FLOW_1_INTAKE:
                setStep(AutoStep.FLOW_1_RETURN);
                followReturnPath(paths[5]);
                break;

            case FLOW_1_RETURN:
                setStep(AutoStep.FLOW_1_SHOOT);
                break;

            case FLOW_1_SHOOT:
                startTransfer(AutoStep.FLOW_1_FEED, FAR_SHOOT_MS);
                break;

            case FLOW_1_FEED:
                setStep(AutoStep.FLOW_2_INTAKE);
                followIntakePath(paths[6]);
                break;

            case FLOW_2_INTAKE:
                setStep(AutoStep.FLOW_2_RETURN);
                followReturnPath(paths[7]);
                break;

            case FLOW_2_RETURN:
                setStep(AutoStep.FLOW_2_SHOOT);
                break;

            case FLOW_2_SHOOT:
                startTransfer(AutoStep.FLOW_2_FEED, FAR_SHOOT_MS);
                break;

            case FLOW_2_FEED:
                setStep(AutoStep.DONE);
                break;

            case DONE:
                setStep(AutoStep.DONE);
                break;

            default:
                setStep(AutoStep.DONE);
                break;
        }
    }

    private boolean isWaitingForShooter() {
        if (dryRun) {
            return false;
        }

        return autoState.getStep() == AutoStep.PRELOAD_SHOOT
                || autoState.getStep() == AutoStep.SPIKE_SHOOT
                || autoState.getStep() == AutoStep.HUMAN_PLAYER_SHOOT
                || autoState.getStep() == AutoStep.FLOW_1_SHOOT
                || autoState.getStep() == AutoStep.FLOW_2_SHOOT;
    }

    private void startFeedWhenShooterReady() {
        if (robot.isShooterReady() || autoState.elapsedMs() >= SHOOTER_WAIT_TIMEOUT_MS) {
            startNextStep();
        }
    }

    private double dryRunStepWaitMs() {
        switch (autoState.getStep()) {
            case PRELOAD_FEED:
                return FAR_PRELOAD_SHOOT_MS;
            case SPIKE_FEED:
            case HUMAN_PLAYER_FEED:
            case FLOW_1_FEED:
            case FLOW_2_FEED:
                return FAR_SHOOT_MS;
            case HUMAN_PLAYER_CONFIRM_INTAKE:
                return FAR_HUMAN_PLAYER_CONFIRM_INTAKE_MS;
            default:
                return 0.0;
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
        return AllianceUtil.mirrorForAlliance(bluePose, isBlueAlliance());
    }

    private void setStep(AutoStep step) {
        autoState.setStep(step);
    }

    private void startTransfer(AutoStep nextStep, double timeoutMs) {
        setStep(nextStep);
        if (!dryRun) {
            robot.transferFor(timeoutMs);
        }
    }

    private void followIntakePath(PathChain path) {
        if (dryRun) {
            robot.followPath(path);
        } else {
            robot.followPathAndIntake(path);
        }
    }

    private void followReturnPath(PathChain path) {
        if (dryRun) {
            robot.followPath(path);
        } else {
            robot.followPathAndIntakeFor(path, FAR_RETURN_INTAKE_MS);
        }
    }

    private void intakeFor(double timeoutMs) {
        if (!dryRun) {
            robot.intakeFor(timeoutMs);
        }
    }

    private String opModeName() {
        return isBlueAlliance() ? "PP Move Only Blue" : "PP Move Only Red";
    }
}
