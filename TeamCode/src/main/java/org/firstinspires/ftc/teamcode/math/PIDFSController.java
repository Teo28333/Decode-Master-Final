package org.firstinspires.ftc.teamcode.math;

import static org.firstinspires.ftc.teamcode.subsystems.constant.ShooterConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * PIDF + kS velocity controller for the shooter flywheel.
 *
 * Tuning order: kF → kS → kP → kD → kI
 *
 *   kF — velocity feedforward (power per RPM unit)
 *   kS — static friction compensation (minimum power to overcome friction)
 *   kP — proportional gain
 *   kD — derivative gain (damping)
 *   kI — integral gain (only if persistent steady-state error remains)
 */
public class PIDFSController {

    // ── Constants ─────────────────────────────────────────────────────────────
    private static final double NOMINAL_VOLTAGE = 12.0;

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final VoltageSensor voltageSensor;

    // ── State ─────────────────────────────────────────────────────────────────
    private double integral   = 0.0;
    private double lastError  = 0.0;
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

        // If target is zero, reset state and coast
        if (targetVel == 0.0) {
            reset();
            return 0.0;
        }

        double error      = targetVel - currentVel;
        double derivative = (error - lastError) / dt;

        // Anti-windup: only integrate when P term is not already saturating output
        if (Math.abs(kP * error) < 1.0) {
            integral = clamp(integral + error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
        }

        // Voltage compensation — scale output to compensate for battery sag
        double voltage      = voltageSensor.getVoltage();
        double voltageScale = voltage > 0.1 ? NOMINAL_VOLTAGE / voltage : 1.0;

        // Feedforward + static friction
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
