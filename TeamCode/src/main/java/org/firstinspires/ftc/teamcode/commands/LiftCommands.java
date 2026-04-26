package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.PtoSS;

public class LiftCommands {

    // ── Subsystem ─────────────────────────────────────────────────────────────
    private final PtoSS pto;

    // ── State ─────────────────────────────────────────────────────────────────
    private Mode    currentMode = Mode.IDLE;
    private boolean hasLifted   = false;

    private enum Mode {
        IDLE,
        ENGAGING,
        LIFTING,
        DISENGAGING
    }

    // ── Constructor ───────────────────────────────────────────────────────────
    public LiftCommands(PtoSS pto) {
        this.pto = pto;
    }

    // ── Loop method ───────────────────────────────────────────────────────────

    /** Call every loop alongside pto.update(). */
    public void update() {
        switch (currentMode) {
            case ENGAGING:
                pto.engagePtoCMD();
                setMode(Mode.LIFTING);
                break;

            case LIFTING:
                pto.activateLiftCMD();
                break;

            case DISENGAGING:
                pto.disengagePtoCMD();
                setMode(Mode.IDLE);
                break;

            case IDLE:
            default:
                break;
        }
    }

    // ── Command triggers ──────────────────────────────────────────────────────

    /**
     * Engage the PTO and begin lifting.
     * No-op if the lift has already been used this match.
     */
    public void lift() {
        if (hasLifted) return;
        hasLifted = true;
        setMode(Mode.ENGAGING);
    }

    /**
     * Disengage the PTO gracefully through the state machine.
     * Does NOT clear hasLifted.
     */
    public void disengage() {
        setMode(Mode.DISENGAGING);
    }

    /**
     * Immediately disengage the PTO, bypassing the state machine.
     * Use for emergency stops or end-of-match cleanup.
     * Does NOT clear hasLifted.
     */
    public void abort() {
        pto.disengagePtoCMD();
        setMode(Mode.IDLE);
    }

    // ── Getters ───────────────────────────────────────────────────────────────
    public boolean isLifting()     { return currentMode == Mode.LIFTING;     }
    public boolean isDisengaging() { return currentMode == Mode.DISENGAGING; }
    public boolean hasLifted()     { return hasLifted;                       }

    // ── Helpers ───────────────────────────────────────────────────────────────
    private void setMode(Mode mode) {
        currentMode = mode;
    }
}