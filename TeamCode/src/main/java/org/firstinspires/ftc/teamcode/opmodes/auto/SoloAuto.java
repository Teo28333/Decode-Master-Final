package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.RobotAuton;

abstract class SoloAuto extends OpMode {
    private RobotAuton robot;
    private Paths paths;
    private final AutoStateMachine<AutoStep> autoState = new AutoStateMachine<>(AutoStep.PRELOAD_SPIN_UP);
    private boolean dryRun = false;
    private boolean lastX = false;

    private enum AutoStep {
        PRELOAD_SPIN_UP,
        PRELOAD_SHOOT,
        PRELOAD_FEED,
        BOTTOM_INTAKE,
        BOTTOM_RETURN,
        BOTTOM_SHOOT,
        BOTTOM_FEED,
        MIDDLE_INTAKE,
        MIDDLE_RETURN,
        MIDDLE_SHOOT,
        MIDDLE_FEED,
        TOP_INTAKE,
        TOP_SWEEP,
        TOP_RETURN,
        TOP_SHOOT,
        TOP_FEED,
        FAR_INTAKE,
        FAR_CONFIRM_INTAKE,
        FAR_RETURN,
        FAR_SHOOT,
        FAR_FEED,
        DONE
    }

    protected abstract boolean isBlueAlliance();

    @Override
    public void init() {
        robot = new RobotAuton(hardwareMap, telemetry, isBlueAlliance());
        paths = new Paths();
        robot.start(alliancePose(new Pose(56.000, 8.500, Math.toRadians(180))));

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
        telemetry.addData("Path timeout ms", SOLO_PATH_STEP_TIMEOUT_MS);
        telemetry.addData("Shooter timeout ms", SHOOTER_WAIT_TIMEOUT_MS);
        telemetry.update();
    }

    @Override
    public void start() {
        setStep(AutoStep.PRELOAD_SPIN_UP);
    }

    @Override
    public void loop() {
        robot.update();

        if (autoState.getStep() == AutoStep.PRELOAD_SPIN_UP) {
            if ((dryRun && autoState.elapsedMs() >= SOLO_PRELOAD_SPIN_UP_MS)
                    || (autoState.elapsedMs() >= SOLO_PRELOAD_SPIN_UP_MS && robot.isShooterReady())
                    || autoState.elapsedMs() >= SOLO_PRELOAD_SPIN_UP_MS + SHOOTER_WAIT_TIMEOUT_MS) {
                setStep(AutoStep.PRELOAD_SHOOT);
            }
        } else if (isWaitingForShooter()) {
            startFeedWhenShooterReady();
        } else if (robot.isBusy() && autoState.elapsedMs() >= SOLO_PATH_STEP_TIMEOUT_MS) {
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

    private void startNextStep() {
        switch (autoState.getStep()) {
            case PRELOAD_SHOOT:
                startTransfer(AutoStep.PRELOAD_FEED, SOLO_PRELOAD_SHOOT_MS);
                break;

            case PRELOAD_FEED:
                setStep(AutoStep.BOTTOM_INTAKE);
                followIntakePath(paths.bottomIntake);
                break;

            case BOTTOM_INTAKE:
                setStep(AutoStep.BOTTOM_RETURN);
                followReturnPath(paths.bottomReturn);
                break;

            case BOTTOM_RETURN:
                setStep(AutoStep.BOTTOM_SHOOT);
                break;

            case BOTTOM_SHOOT:
                startTransfer(AutoStep.BOTTOM_FEED, SOLO_SHOOT_MS);
                break;

            case BOTTOM_FEED:
                setStep(AutoStep.MIDDLE_INTAKE);
                followIntakePath(paths.middleIntake);
                break;

            case MIDDLE_INTAKE:
                setStep(AutoStep.MIDDLE_RETURN);
                followReturnPath(paths.middleReturn);
                break;

            case MIDDLE_RETURN:
                setStep(AutoStep.MIDDLE_SHOOT);
                break;

            case MIDDLE_SHOOT:
                startTransfer(AutoStep.MIDDLE_FEED, SOLO_SHOOT_MS);
                break;

            case MIDDLE_FEED:
                setStep(AutoStep.TOP_INTAKE);
                followIntakePath(paths.topIntake);
                break;

            case TOP_INTAKE:
                setStep(AutoStep.TOP_SWEEP);
                followIntakePath(paths.topSweep);
                break;

            case TOP_SWEEP:
                setStep(AutoStep.TOP_RETURN);
                followReturnPath(paths.topReturn);
                break;

            case TOP_RETURN:
                setStep(AutoStep.TOP_SHOOT);
                break;

            case TOP_SHOOT:
                startTransfer(AutoStep.TOP_FEED, SOLO_SHOOT_MS);
                break;

            case TOP_FEED:
                setStep(AutoStep.FAR_INTAKE);
                followIntakePath(paths.farIntake);
                break;

            case FAR_INTAKE:
                setStep(AutoStep.FAR_CONFIRM_INTAKE);
                intakeFor(SOLO_HUMAN_PLAYER_CONFIRM_INTAKE_MS);
                break;

            case FAR_CONFIRM_INTAKE:
                setStep(AutoStep.FAR_RETURN);
                followReturnPath(paths.farReturn);
                break;

            case FAR_RETURN:
                setStep(AutoStep.FAR_SHOOT);
                break;

            case FAR_SHOOT:
                startTransfer(AutoStep.FAR_FEED, SOLO_SHOOT_MS);
                break;

            case FAR_FEED:
            case DONE:
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
                || autoState.getStep() == AutoStep.BOTTOM_SHOOT
                || autoState.getStep() == AutoStep.MIDDLE_SHOOT
                || autoState.getStep() == AutoStep.TOP_SHOOT
                || autoState.getStep() == AutoStep.FAR_SHOOT;
    }

    private void startFeedWhenShooterReady() {
        if (robot.isShooterReady() || autoState.elapsedMs() >= SHOOTER_WAIT_TIMEOUT_MS) {
            startNextStep();
        }
    }

    private double dryRunStepWaitMs() {
        switch (autoState.getStep()) {
            case PRELOAD_FEED:
                return SOLO_PRELOAD_SHOOT_MS;
            case BOTTOM_FEED:
            case MIDDLE_FEED:
            case TOP_FEED:
            case FAR_FEED:
                return SOLO_SHOOT_MS;
            case FAR_CONFIRM_INTAKE:
                return SOLO_HUMAN_PLAYER_CONFIRM_INTAKE_MS;
            default:
                return 0.0;
        }
    }

    private void startTransfer(AutoStep nextStep, double timeoutMs) {
        setStep(nextStep);
        if (!dryRun) {
            robot.transferFor(timeoutMs);
        }
    }

    private void intakeFor(double timeoutMs) {
        if (!dryRun) {
            robot.intakeFor(timeoutMs);
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
            robot.followPathAndIntakeFor(path, SOLO_RETURN_INTAKE_MS);
        }
    }

    private Pose alliancePose(Pose bluePose) {
        return AllianceUtil.mirrorForAlliance(bluePose, isBlueAlliance());
    }

    private double allianceHeading(double blueHeading) {
        return isBlueAlliance() ? blueHeading : Math.PI - blueHeading;
    }

    private void setStep(AutoStep step) {
        autoState.setStep(step);
    }

    private String opModeName() {
        return isBlueAlliance() ? "PP Solo Blue" : "PP Solo Red";
    }

    private class Paths {
        private final PathChain bottomIntake;
        private final PathChain bottomReturn;
        private final PathChain middleIntake;
        private final PathChain middleReturn;
        private final PathChain topIntake;
        private final PathChain topSweep;
        private final PathChain topReturn;
        private final PathChain farIntake;
        private final PathChain farReturn;

        private Paths() {
            bottomIntake = tangentCurve(
                    new Pose(56.000, 8.500, Math.toRadians(180)),
                    new Pose(35.979, 16.565),
                    new Pose(9.706, 11.041),
                    false
            );
            bottomReturn = tangentLine(new Pose(9.706, 11.041), new Pose(49.150, 12.957), true);
            middleIntake = tangentCurve(
                    new Pose(49.150, 12.957),
                    new Pose(46.584, 34.619),
                    new Pose(10.204, 34.945),
                    false
            );
            middleReturn = tangentLine(new Pose(10.204, 34.945), new Pose(49.373, 13.177), true);
            topIntake = tangentCurve(
                    new Pose(49.373, 13.177),
                    new Pose(48.048, 57.618),
                    new Pose(16.179, 58.530),
                    false
            );
            topSweep = linearHeadingCurve(
                    new Pose(16.179, 58.530),
                    new Pose(24.575, 63.396),
                    new Pose(16.089, 68.034),
                    Math.toRadians(180),
                    Math.toRadians(90)
            );
            topReturn = linearHeadingCurve(
                    new Pose(16.089, 68.034),
                    new Pose(45.326, 68.953),
                    new Pose(47.177, 80.389),
                    Math.toRadians(90),
                    Math.toRadians(180)
            );
            farIntake = tangentLine(new Pose(47.177, 80.389), new Pose(20.236, 82.382), false);
            farReturn = tangentCurve(
                    new Pose(20.236, 82.382),
                    new Pose(38.189, 93.970),
                    new Pose(40.455, 124.491),
                    true
            );
        }
    }

    private PathChain tangentLine(Pose blueStart, Pose blueEnd, boolean reversed) {
        PathBuilder builder = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(alliancePose(blueStart), alliancePose(blueEnd)))
                .setTangentHeadingInterpolation();

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    private PathChain tangentCurve(Pose blueStart, Pose blueControl, Pose blueEnd, boolean reversed) {
        PathBuilder builder = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(
                        alliancePose(blueStart),
                        alliancePose(blueControl),
                        alliancePose(blueEnd)
                ))
                .setTangentHeadingInterpolation();

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    private PathChain linearHeadingCurve(Pose blueStart, Pose blueControl, Pose blueEnd,
                                         double blueStartHeading, double blueEndHeading) {
        Follower follower = robot.getFollower();
        return follower.pathBuilder()
                .addPath(new BezierCurve(
                        alliancePose(blueStart),
                        alliancePose(blueControl),
                        alliancePose(blueEnd)
                ))
                .setLinearHeadingInterpolation(
                        allianceHeading(blueStartHeading),
                        allianceHeading(blueEndHeading)
                )
                .build();
    }
}
