package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;

public class IntakeCommands {

    // ── Subsystem ─────────────────────────────────────────────────────────────
    private final IntakeSS intake;

    // ── State ─────────────────────────────────────────────────────────────────
    private Mode currentMode = Mode.IDLE;

    private enum Mode {
        IDLE,
        INTAKING,
        OUTTAKING,
        TRANSFERRING
    }

    // ── Constructor ───────────────────────────────────────────────────────────
    public IntakeCommands(IntakeSS intake) {
        this.intake = intake;
    }

    // ── Loop method ───────────────────────────────────────────────────────────

    /**
     * Call every loop alongside intake.update().
     *
     * @param robotX        robot X position (inches)
     * @param robotY        robot Y position (inches)
     * @param shooterReady  true when shooter is at target RPM
     * @param aimedAtTarget true when turret is aimed at goal
     */
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
                break;
        }
    }

    // ── Command triggers ──────────────────────────────────────────────────────

    /** Start intaking. Resets any previous ball detection state. */
    public void intake() {
        intake.resetIntake();
        setMode(Mode.INTAKING);
    }

    /** Start outtaking. */
    public void outtake() {
        setMode(Mode.OUTTAKING);
    }

    /** Begin transferring the ball to the shooter. */
    public void transfer() {
        setMode(Mode.TRANSFERRING);
    }

    /** Stop all intake motion. */
    public void idle() {
        setMode(Mode.IDLE);
    }

    // ── Getters ───────────────────────────────────────────────────────────────
    public boolean isIntaking()     { return currentMode == Mode.INTAKING;     }
    public boolean isOuttaking()    { return currentMode == Mode.OUTTAKING;    }
    public boolean isTransferring() { return currentMode == Mode.TRANSFERRING; }
    public boolean needToTurn()     { return intake.needToTurn();              }

    // ── Helpers ───────────────────────────────────────────────────────────────
    private void setMode(Mode mode) {
        currentMode = mode;
    }
}