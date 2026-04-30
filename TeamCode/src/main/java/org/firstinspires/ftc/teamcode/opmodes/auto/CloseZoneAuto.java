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
    private int selectedGateIntakes = CLOSE_DEFAULT_GATE_INTAKES;
    private int gateIntakeCycle = 0;
    private boolean dryRun = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastX = false;

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
        paths = new Paths();
        robot.start(alliancePose(new Pose(18.603, 120.278, Math.toRadians(-40))));

        telemetry.addLine(opModeName() + " ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        boolean dpadUpPressed = gamepad1.dpad_up && !lastDpadUp;
        boolean dpadDownPressed = gamepad1.dpad_down && !lastDpadDown;
        boolean xPressed = gamepad1.x && !lastX;

        if (dpadUpPressed) {
            selectedGateIntakes = Math.min(CLOSE_MAX_GATE_INTAKES, selectedGateIntakes + 1);
        }
        if (dpadDownPressed) {
            selectedGateIntakes = Math.max(CLOSE_MIN_GATE_INTAKES, selectedGateIntakes - 1);
        }
        if (xPressed) {
            dryRun = !dryRun;
        }

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastX = gamepad1.x;

        telemetry.addLine(opModeName() + " ready");
        telemetry.addData("Step", autoState.getStep());
        telemetry.addData("Gate intakes", selectedGateIntakes);
        telemetry.addData("Dry run", dryRun);
        telemetry.addData("Path timeout ms", PATH_STEP_TIMEOUT_MS);
        telemetry.addData("Shooter timeout ms", SHOOTER_WAIT_TIMEOUT_MS);
        telemetry.update();
    }

    @Override
    public void start() {
        autoState.setStep(AutoStep.GO_TO_SHOOTING_POSE);
        gateIntakeCycle = 0;
        robot.followPath(paths.goToShootingPose);
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
        telemetry.addData("Gate intake cycle", "%d / %d", gateIntakeCycle, selectedGateIntakes);
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
                setStep(AutoStep.MIDDLE_INTAKE);
                followIntakePath(paths.intakeMiddleLine);
                break;

            case MIDDLE_INTAKE:
                setStep(AutoStep.MIDDLE_RETURN);
                followReturnPath(paths.goToShootingPose2);
                break;

            case MIDDLE_RETURN:
                setStep(AutoStep.MIDDLE_SHOOT);
                break;

            case MIDDLE_SHOOT:
                startTransfer(AutoStep.MIDDLE_FEED, CLOSE_SHOOT_MS);
                break;

            case MIDDLE_FEED:
                setStep(AutoStep.GATE_INTAKE);
                gateIntakeCycle = 1;
                followIntakePath(paths.gateIntake1);
                break;

            case GATE_INTAKE:
                setStep(AutoStep.GATE_CONFIRM_INTAKE);
                intakeFor(CLOSE_GATE_CONFIRM_INTAKE_MS);
                break;

            case GATE_CONFIRM_INTAKE:
                setStep(AutoStep.GATE_RETURN);
                followReturnPath(paths.goToShootingPose3);
                break;

            case GATE_RETURN:
                setStep(AutoStep.GATE_SHOOT);
                break;

            case GATE_SHOOT:
                startTransfer(AutoStep.GATE_FEED, CLOSE_SHOOT_MS);
                break;

            case GATE_FEED:
                if (gateIntakeCycle < selectedGateIntakes) {
                    gateIntakeCycle++;
                    setStep(AutoStep.GATE_INTAKE);
                    followIntakePath(paths.gateIntake1);
                } else {
                    setStep(AutoStep.RIGHT_INTAKE);
                    followIntakePath(paths.intakeRightLine);
                }
                break;

            case RIGHT_INTAKE:
                setStep(AutoStep.RIGHT_RETURN);
                followReturnPath(paths.goToShootingPose4);
                break;

            case RIGHT_RETURN:
                setStep(AutoStep.RIGHT_SHOOT);
                break;

            case RIGHT_SHOOT:
                startTransfer(AutoStep.RIGHT_FEED, CLOSE_SHOOT_MS);
                break;

            case RIGHT_FEED:
                setStep(AutoStep.LEAVING_ZONE);
                robot.followPath(paths.leavingZone);
                break;

            case LEAVING_ZONE:
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
                || autoState.getStep() == AutoStep.MIDDLE_SHOOT
                || autoState.getStep() == AutoStep.GATE_SHOOT
                || autoState.getStep() == AutoStep.RIGHT_SHOOT;
    }

    private void startFeedWhenShooterReady() {
        if (robot.isShooterReady() || autoState.elapsedMs() >= SHOOTER_WAIT_TIMEOUT_MS) {
            startNextStep();
        }
    }

    private double dryRunStepWaitMs() {
        switch (autoState.getStep()) {
            case PRELOAD_FEED:
                return CLOSE_PRELOAD_SHOOT_MS;
            case MIDDLE_FEED:
            case GATE_FEED:
            case RIGHT_FEED:
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
