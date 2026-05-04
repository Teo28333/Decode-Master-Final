package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.*;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.RobotAuton;

abstract class CloseZoneAuto extends OpMode {
    private RobotAuton robot;
    private Paths paths;
    private final AutoStateMachine<AutoStep> autoState = new AutoStateMachine<>(AutoStep.GO_TO_SHOOTING_POSE);
    private int selectedGateIntakeShootCycles = CLOSE_DEFAULT_GATE_INTAKES;
    private int gateIntakeShootCycle = 0;
    private boolean dryRun = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastX = false;

    private enum AutoStep {
        GO_TO_SHOOTING_POSE,
        PRELOAD_SHOOT,
        PRELOAD_FEED,
        LEFT_INTAKE,
        LEFT_RETURN,
        LEFT_SHOOT,
        LEFT_FEED,
        MIDDLE_INTAKE,
        MIDDLE_RETURN,
        MIDDLE_SHOOT,
        MIDDLE_FEED,
        GATE_INTAKE,
        GATE_CONFIRM_INTAKE,
        GATE_RETURN,
        GATE_SHOOT,
        GATE_FEED,
        DONE
    }

    protected abstract boolean isBlueAlliance();

    @Override
    public void init() {
        robot = new RobotAuton(hardwareMap, telemetry, isBlueAlliance());
        paths = new Paths();
        robot.start(alliancePose(new Pose(18.571, 117.403, Math.toRadians(-36))));

        telemetry.addLine(opModeName() + " ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        boolean dpadUpPressed = gamepad1.dpad_up && !lastDpadUp;
        boolean dpadDownPressed = gamepad1.dpad_down && !lastDpadDown;
        boolean xPressed = gamepad1.x && !lastX;

        if (dpadUpPressed) {
            selectedGateIntakeShootCycles = clampGateIntakeShootCycles(selectedGateIntakeShootCycles + 1);
        }
        if (dpadDownPressed) {
            selectedGateIntakeShootCycles = clampGateIntakeShootCycles(selectedGateIntakeShootCycles - 1);
        }
        if (xPressed) {
            dryRun = !dryRun;
        }

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastX = gamepad1.x;

        telemetry.addLine(opModeName() + " ready");
        telemetry.addData("Step", autoState.getStep());
        telemetry.addData("Gate intake+shoot chains", selectedGateIntakeShootCycles);
        telemetry.addData("Dry run", dryRun);
        telemetry.addData("Path timeout ms", PATH_STEP_TIMEOUT_MS);
        telemetry.addData("Shooter timeout ms", SHOOTER_WAIT_TIMEOUT_MS);
        telemetry.update();
    }

    @Override
    public void start() {
        autoState.setStep(AutoStep.GO_TO_SHOOTING_POSE);
        selectedGateIntakeShootCycles = clampGateIntakeShootCycles(selectedGateIntakeShootCycles);
        gateIntakeShootCycle = 0;
        robot.followPath(paths.closezone);
    }

    @Override
    public void loop() {
        robot.update();

        if (isWaitingForShooter()) {
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
        telemetry.addData("Gate intake+shoot chain", "%d / %d", gateIntakeShootCycle, selectedGateIntakeShootCycles);
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
            case GO_TO_SHOOTING_POSE:
                setStep(AutoStep.PRELOAD_SHOOT);
                break;

            case PRELOAD_SHOOT:
                startTransfer(AutoStep.PRELOAD_FEED, CLOSE_PRELOAD_SHOOT_MS);
                break;

            case PRELOAD_FEED:
                setStep(AutoStep.LEFT_INTAKE);
                followIntakePath(paths.leftIntake);
                break;

            case LEFT_INTAKE:
                setStep(AutoStep.LEFT_RETURN);
                followReturnPath(paths.leftReturn);
                break;

            case LEFT_RETURN:
                setStep(AutoStep.LEFT_SHOOT);
                break;

            case LEFT_SHOOT:
                startTransfer(AutoStep.LEFT_FEED, CLOSE_SHOOT_MS);
                break;

            case LEFT_FEED:
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
                startTransfer(AutoStep.MIDDLE_FEED, CLOSE_SHOOT_MS);
                break;

            case MIDDLE_FEED:
                gateIntakeShootCycle = 0;
                startNextGateIntakeShootCycle();
                break;

            case GATE_INTAKE:
                setStep(AutoStep.GATE_CONFIRM_INTAKE);
                intakeFor(CLOSE_GATE_CONFIRM_INTAKE_MS);
                break;

            case GATE_CONFIRM_INTAKE:
                setStep(AutoStep.GATE_RETURN);
                followReturnPath(paths.gateReturn);
                break;

            case GATE_RETURN:
                setStep(AutoStep.GATE_SHOOT);
                break;

            case GATE_SHOOT:
                startTransfer(AutoStep.GATE_FEED, CLOSE_SHOOT_MS);
                break;

            case GATE_FEED:
                startNextGateIntakeShootCycle();
                break;

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
                || autoState.getStep() == AutoStep.LEFT_SHOOT
                || autoState.getStep() == AutoStep.MIDDLE_SHOOT
                || autoState.getStep() == AutoStep.GATE_SHOOT;
    }

    private void startFeedWhenShooterReady() {
        if (robot.isShooterReady() || autoState.elapsedMs() >= SHOOTER_WAIT_TIMEOUT_MS) {
            startNextStep();
        }
    }

    private void startNextGateIntakeShootCycle() {
        if (gateIntakeShootCycle >= selectedGateIntakeShootCycles) {
            setStep(AutoStep.DONE);
            return;
        }

        gateIntakeShootCycle++;
        setStep(AutoStep.GATE_INTAKE);
        followIntakePath(paths.gateIntake);
    }

    private double dryRunStepWaitMs() {
        switch (autoState.getStep()) {
            case PRELOAD_FEED:
                return CLOSE_PRELOAD_SHOOT_MS;
            case LEFT_FEED:
            case MIDDLE_FEED:
            case GATE_FEED:
                return CLOSE_SHOOT_MS;
            case GATE_CONFIRM_INTAKE:
                return CLOSE_GATE_CONFIRM_INTAKE_MS;
            default:
                return 0.0;
        }
    }

    private Pose alliancePose(Pose bluePose) {
        return AllianceUtil.mirrorForAlliance(bluePose, isBlueAlliance());
    }

    private double allianceHeading(double blueHeading) {
        return isBlueAlliance() ? blueHeading : Math.PI - blueHeading;
    }

    private int clampGateIntakeShootCycles(int cycles) {
        return Math.max(CLOSE_MIN_GATE_INTAKES, Math.min(CLOSE_MAX_GATE_INTAKES, cycles));
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
            robot.followPathAndIntakeFor(path, CLOSE_RETURN_INTAKE_MS);
        }
    }

    private void intakeFor(double timeoutMs) {
        if (!dryRun) {
            robot.intakeFor(timeoutMs);
        }
    }

    private String opModeName() {
        return isBlueAlliance() ? "PP Close Zone Blue" : "PP Close Zone Red";
    }

    private class Paths {
        private final PathChain closezone;
        private final PathChain leftIntake;
        private final PathChain leftReturn;
        private final PathChain middleIntake;
        private final PathChain middleReturn;
        private final PathChain gateIntake;
        private final PathChain gateReturn;

        private Paths() {
            closezone = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierLine(
                                    alliancePose(new Pose(18.571, 117.403)),
                                    alliancePose(new Pose(40.000, 95.000))
                            )
                    )
                    .setLinearHeadingInterpolation(
                            allianceHeading(Math.toRadians(-36)),
                            allianceHeading(Math.toRadians(180))
                    )
                    .build();

            leftIntake = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierLine(
                                    alliancePose(new Pose(40.000, 95.000)),
                                    alliancePose(new Pose(19.000, 78.000))
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            leftReturn = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierLine(
                                    alliancePose(new Pose(19.000, 78.000)),
                                    alliancePose(new Pose(47.000, 82.000))
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            middleIntake = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    alliancePose(new Pose(47.000, 82.000)),
                                    alliancePose(new Pose(38.404, 63.738)),
                                    alliancePose(new Pose(16.296, 53.967))
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            middleReturn = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierLine(
                                    alliancePose(new Pose(16.296, 53.967)),
                                    alliancePose(new Pose(49.000, 84.000))
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            gateIntake = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    alliancePose(new Pose(49.000, 84.000)),
                                    alliancePose(new Pose(26.477, 60.234)),
                                    alliancePose(new Pose(10.622, 56.370))
                            )
                    )
                    .setLinearHeadingInterpolation(
                            allianceHeading(Math.toRadians(223)),
                            allianceHeading(Math.toRadians(150))
                    )
                    .build();

            gateReturn = robot.getFollower().pathBuilder()
                    .addPath(
                            new BezierLine(
                                    alliancePose(new Pose(10.622, 56.370)),
                                    alliancePose(new Pose(47.000, 82.000))
                            )
                    )
                    .setLinearHeadingInterpolation(
                            allianceHeading(Math.toRadians(150)),
                            allianceHeading(Math.toRadians(223))
                    )
                    .setReversed()
                    .build();
        }
    }
}
