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
                break;

            case IDLE:
            default:
                intake.stop();   // ← THIS is what was missing
                break;
        }
    }

    public void intake() {
        if (currentMode != Mode.INTAKING) {
            intake.resetIntake();
        }
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
    public IntakeSS.BallState getBallState() { return intake.getBallState();   }
    public IntakeSS.TransferState getTransferState() { return intake.getTransferState(); }
    public boolean isFull() { return intake.isFull(); }
    public String getState() {
        if (currentMode == Mode.TRANSFERRING) {
            return currentMode + " / " + intake.getTransferState();
        }
        if (currentMode == Mode.INTAKING) {
            return currentMode + " / " + intake.getBallState();
        }
        return currentMode.toString();
    }

    private void setMode(Mode mode) {
        currentMode = mode;
    }
}
