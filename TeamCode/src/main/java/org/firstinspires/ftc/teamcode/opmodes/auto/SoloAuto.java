package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.*;

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
        GO_TO_HUMAN_ZONE,
        INTAKE_HUMAN_ZONE,
        HUMAN_ZONE_CONFIRM_INTAKE,
        GO_TO_SHOOT_FROM_HUMAN_ZONE,
        HUMAN_ZONE_SHOOT,
        HUMAN_ZONE_FEED,
        INTAKE_LEFT_LINE,
        GO_TO_SHOOT_FROM_LEFT_LINE,
        LEFT_LINE_SHOOT,
        LEFT_LINE_FEED,
        INTAKE_MIDDLE_LINE,
        CLEAR_CLASSIFIER,
        GO_TO_SHOOT_FROM_MIDDLE_LINE,
        MIDDLE_LINE_SHOOT,
        MIDDLE_LINE_FEED,
        INTAKE_RIGHT_LINE,
        GO_TO_SHOOT_FROM_RIGHT_LINE,
        RIGHT_LINE_SHOOT,
        RIGHT_LINE_FEED,
        DONE
    }

    protected abstract boolean isBlueAlliance();

    @Override
    public void init() {
        robot = new RobotAuton(hardwareMap, telemetry, isBlueAlliance());
        paths = new Paths();
        robot.start(alliancePose(new Pose(56.000, 8.500, Math.toRadians(150))));

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
                setStep(AutoStep.GO_TO_HUMAN_ZONE);
                robot.followPath(paths.goToHumanZone);
                break;

            case GO_TO_HUMAN_ZONE:
                setStep(AutoStep.INTAKE_HUMAN_ZONE);
                followIntakePath(paths.intakeHumanZone);
                break;

            case INTAKE_HUMAN_ZONE:
                setStep(AutoStep.HUMAN_ZONE_CONFIRM_INTAKE);
                intakeFor(SOLO_HUMAN_PLAYER_CONFIRM_INTAKE_MS);
                break;

            case HUMAN_ZONE_CONFIRM_INTAKE:
                setStep(AutoStep.GO_TO_SHOOT_FROM_HUMAN_ZONE);
                followReturnPath(paths.goToShootFromHumanZone);
                break;

            case GO_TO_SHOOT_FROM_HUMAN_ZONE:
                setStep(AutoStep.HUMAN_ZONE_SHOOT);
                break;

            case HUMAN_ZONE_SHOOT:
                startTransfer(AutoStep.HUMAN_ZONE_FEED, SOLO_SHOOT_MS);
                break;

            case HUMAN_ZONE_FEED:
                setStep(AutoStep.INTAKE_LEFT_LINE);
                followIntakePath(paths.intakeLeftLine);
                break;

            case INTAKE_LEFT_LINE:
                setStep(AutoStep.GO_TO_SHOOT_FROM_LEFT_LINE);
                followReturnPath(paths.goToShootFromLeftLine);
                break;

            case GO_TO_SHOOT_FROM_LEFT_LINE:
                setStep(AutoStep.LEFT_LINE_SHOOT);
                break;

            case LEFT_LINE_SHOOT:
                startTransfer(AutoStep.LEFT_LINE_FEED, SOLO_SHOOT_MS);
                break;

            case LEFT_LINE_FEED:
                setStep(AutoStep.INTAKE_MIDDLE_LINE);
                followIntakePath(paths.intakeMiddleLine);
                break;

            case INTAKE_MIDDLE_LINE:
                setStep(AutoStep.CLEAR_CLASSIFIER);
                followIntakePath(paths.clearClassifier);
                break;

            case CLEAR_CLASSIFIER:
                setStep(AutoStep.GO_TO_SHOOT_FROM_MIDDLE_LINE);
                followReturnPath(paths.goToShootFromMiddleLine);
                break;

            case GO_TO_SHOOT_FROM_MIDDLE_LINE:
                setStep(AutoStep.MIDDLE_LINE_SHOOT);
                break;

            case MIDDLE_LINE_SHOOT:
                startTransfer(AutoStep.MIDDLE_LINE_FEED, SOLO_SHOOT_MS);
                break;

            case MIDDLE_LINE_FEED:
                setStep(AutoStep.INTAKE_RIGHT_LINE);
                followIntakePath(paths.intakeRightLine);
                break;

            case INTAKE_RIGHT_LINE:
                setStep(AutoStep.GO_TO_SHOOT_FROM_RIGHT_LINE);
                followReturnPath(paths.goToShootFromRightLine);
                break;

            case GO_TO_SHOOT_FROM_RIGHT_LINE:
                setStep(AutoStep.RIGHT_LINE_SHOOT);
                break;

            case RIGHT_LINE_SHOOT:
                startTransfer(AutoStep.RIGHT_LINE_FEED, SOLO_SHOOT_MS);
                break;

            case RIGHT_LINE_FEED:
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
                || autoState.getStep() == AutoStep.HUMAN_ZONE_SHOOT
                || autoState.getStep() == AutoStep.LEFT_LINE_SHOOT
                || autoState.getStep() == AutoStep.MIDDLE_LINE_SHOOT
                || autoState.getStep() == AutoStep.RIGHT_LINE_SHOOT;
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
            case HUMAN_ZONE_FEED:
            case LEFT_LINE_FEED:
            case MIDDLE_LINE_FEED:
            case RIGHT_LINE_FEED:
                return SOLO_SHOOT_MS;
            case HUMAN_ZONE_CONFIRM_INTAKE:
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
        private final PathChain goToHumanZone;
        private final PathChain intakeHumanZone;
        private final PathChain goToShootFromHumanZone;
        private final PathChain intakeLeftLine;
        private final PathChain goToShootFromLeftLine;
        private final PathChain intakeMiddleLine;
        private final PathChain clearClassifier;
        private final PathChain goToShootFromMiddleLine;
        private final PathChain intakeRightLine;
        private final PathChain goToShootFromRightLine;

        private Paths() {
            goToHumanZone = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierLine(
                                    alliancePose(new Pose(56.000, 8.500)),
                                    alliancePose(new Pose(9.000, 25.000))
                            )
                    )
                    .setLinearHeadingInterpolation(
                            allianceHeading(Math.toRadians(150)),
                            allianceHeading(Math.toRadians(270))
                    )
                    .setReversed()
                    .build();

            intakeHumanZone = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierLine(
                                    alliancePose(new Pose(9.000, 25.000)),
                                    alliancePose(new Pose(9.000, 10.000))
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            goToShootFromHumanZone = linearHeadingLine(
                    new Pose(9.000, 10.000),
                    new Pose(49.445, 13.872),
                    Math.toRadians(270),
                    Math.toRadians(100),
                    false
            );

            intakeLeftLine = tangentCurve(
                    new Pose(49.445, 13.872),
                    new Pose(45.219, 34.889),
                    new Pose(14.000, 36.000),
                    false
            );
            goToShootFromLeftLine = tangentLine(new Pose(14.000, 36.000), new Pose(49.363, 13.810), true);
            intakeMiddleLine = tangentCurve(
                    new Pose(49.363, 13.810),
                    new Pose(41.217, 60.900),
                    new Pose(13.573, 59.041),
                    false
            );
            clearClassifier = linearHeadingCurve(
                    new Pose(13.573, 59.041),
                    new Pose(21.318, 59.861),
                    new Pose(14.168, 68.544),
                    Math.toRadians(180),
                    Math.toRadians(-90),
                    true
            );
            goToShootFromMiddleLine = linearHeadingCurve(
                    new Pose(14.168, 68.544),
                    new Pose(41.209, 58.996),
                    new Pose(47.000, 82.000),
                    Math.toRadians(-90),
                    Math.toRadians(180),
                    true
            );
            intakeRightLine = tangentLine(new Pose(47.000, 82.000), new Pose(15.000, 82.000), false);
            goToShootFromRightLine = tangentLine(new Pose(15.000, 82.000), new Pose(47.000, 82.000), true);
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

    private PathChain linearHeadingLine(Pose blueStart, Pose blueEnd,
                                        double blueStartHeading, double blueEndHeading,
                                        boolean reversed) {
        PathBuilder builder = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(alliancePose(blueStart), alliancePose(blueEnd)))
                .setLinearHeadingInterpolation(
                        allianceHeading(blueStartHeading),
                        allianceHeading(blueEndHeading)
                );

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    private PathChain linearHeadingCurve(Pose blueStart, Pose blueControl, Pose blueEnd,
                                         double blueStartHeading, double blueEndHeading,
                                         boolean reversed) {
        PathBuilder builder = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(
                        alliancePose(blueStart),
                        alliancePose(blueControl),
                        alliancePose(blueEnd)
                ))
                .setLinearHeadingInterpolation(
                        allianceHeading(blueStartHeading),
                        allianceHeading(blueEndHeading)
                );

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }
}
