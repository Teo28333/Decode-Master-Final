package org.firstinspires.ftc.teamcode.math;

import static org.firstinspires.ftc.teamcode.subsystems.constant.ShooterConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class PIDFSController {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final VoltageSensor voltageSensor;

    // ── State ─────────────────────────────────────────────────────────────────
    private double integral  = 0.0;
    private double lastError = 0.0;
    private long   lastTimeNs = -1L;

    // ── Constructor ───────────────────────────────────────────────────────────
    public PIDFSController(HardwareMap hardwareMap) {
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    // ── Main loop ─────────────────────────────────────────────────────────────
    public double calculate(double targetVel, double currentVel) {
        long nowNs = System.nanoTime();

        // First call — seed state and return 0
        if (lastTimeNs < 0) {
            lastTimeNs = nowNs;
            lastError  = targetVel - currentVel;
            return 0.0;
        }

        double dt = (nowNs - lastTimeNs) * 1e-9;
        lastTimeNs = nowNs;
        if (dt <= 0) return 0.0;

        // If target is zero, reset and coast
        if (targetVel == 0.0) {
            reset();
            return 0.0;
        }

        double error      = targetVel - currentVel;
        double derivative = (error - lastError) / dt;

        // Anti-windup: only integrate when output is not saturated
        if (Math.abs(kP * error) < 1.0) {
            integral = clamp(integral + error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
        }

        // Voltage compensation
        double voltage      = voltageSensor.getVoltage();
        double voltageScale = voltage > 0.1 ? nominalVoltage / voltage : 1.0;

        // Feedforward
        double ff = kF * targetVel * voltageScale;
        double ks = Math.copySign(kS, targetVel);

        double output = kP * error
                + kI * integral
                + kD * derivative
                + ff
                + ks;

        lastError = error;
        return clamp(output, -1.0, 1.0);
    }

    public void reset() {
        integral   = 0.0;
        lastError  = 0.0;
        lastTimeNs = -1L;
    }

    // ── Utility ───────────────────────────────────────────────────────────────
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}