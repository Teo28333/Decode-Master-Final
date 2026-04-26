package org.firstinspires.ftc.teamcode.math;

import static org.firstinspires.ftc.teamcode.subsystems.constant.PtoConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Simple PIDF position controller for the PTO lift mechanism.
 * Uses encoder ticks as the position unit.
 */
public class PIDFController {

    // ── State ─────────────────────────────────────────────────────────────────
    private double integral  = 0.0;
    private double lastError = 0.0;

    // ── Constructor ───────────────────────────────────────────────────────────
    public PIDFController(HardwareMap hardwareMap) {
        // No hardware needed for a basic PIDF position controller
    }

    // ── Main loop ─────────────────────────────────────────────────────────────
    public double calculate(double target, double current) {
        double error      = target - current;
        integral         += error;
        double derivative = error - lastError;
        lastError         = error;

        double output = kP * error
                      + kI * integral
                      + kD * derivative
                      + kF * target;

        return Math.max(-1.0, Math.min(1.0, output));
    }

    public void reset() {
        integral  = 0.0;
        lastError = 0.0;
    }
}
