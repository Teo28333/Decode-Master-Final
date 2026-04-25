package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;

public class IntakeCommands {

    private final IntakeSS intake;
    private Mode currentMode = Mode.IDLE;

    private enum Mode {
        IDLE,
        INTAKING,
        OUTTAKING,
        TRANSFERRING
    }

    public IntakeCommands(IntakeSS intake) {
        this.intake = intake;
    }

    public void update(double robotX, double robotY,
                       boolean shooterReady, boolean aimedAtTarget) {
        switch (currentMode) {
            case INTAKING:
                intake.intakeCMD();
                break;

            case OUTTAKING:
                intake.outtakeCMD();
                break;

            case TRANSFERRING:
                intake.transferCMD(robotX, robotY, shooterReady, aimedAtTarget);
                if (shooterReady && aimedAtTarget) setMode(Mode.IDLE);
                break;

            case IDLE:
            default:
                intake.stop();   // ← THIS is what was missing
                break;
        }
    }

    public void intake() {
        intake.resetIntake();
        setMode(Mode.INTAKING);
    }

    public void outtake() {
        setMode(Mode.OUTTAKING);
    }

    public void transfer() {
        setMode(Mode.TRANSFERRING);
    }

    public void idle() {
        setMode(Mode.IDLE);
    }

    public boolean isIntaking()     { return currentMode == Mode.INTAKING;     }
    public boolean isOuttaking()    { return currentMode == Mode.OUTTAKING;    }
    public boolean isTransferring() { return currentMode == Mode.TRANSFERRING; }
    public boolean needToTurn()     { return intake.needToTurn();              }

    private void setMode(Mode mode) {
        currentMode = mode;
    }
}