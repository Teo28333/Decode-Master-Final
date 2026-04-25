package org.firstinspires.ftc.teamcode.math;

import static org.firstinspires.ftc.teamcode.subsystems.constant.ShooterConstants.MAX_INTEGRAL;
import static org.firstinspires.ftc.teamcode.subsystems.constant.PtoConstants.kD;
import static org.firstinspires.ftc.teamcode.subsystems.constant.PtoConstants.kF;
import static org.firstinspires.ftc.teamcode.subsystems.constant.PtoConstants.kI;
import static org.firstinspires.ftc.teamcode.subsystems.constant.PtoConstants.kP;
import static org.firstinspires.ftc.teamcode.subsystems.constant.PtoConstants.nominalVoltage;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class PIDFController {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final VoltageSensor voltageSensor;

    // ── State ─────────────────────────────────────────────────────────────────
    private double integral  = 0.0;
    private double lastError = 0.0;
    private long   lastTimeNs = -1L;

    // ── Constructor ───────────────────────────────────────────────────────────
    public PIDFController(HardwareMap hardwareMap) {
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    // ── Main loop ─────────────────────────────────────────────────────────────
    public double calculate(double targetPos, double currentPos) {
        long nowNs = System.nanoTime();

        // First call — seed state and return 0
        if (lastTimeNs < 0) {
            lastTimeNs = nowNs;
            lastError  = targetPos - currentPos;
            return 0.0;
        }

        double dt = (nowNs - lastTimeNs) * 1e-9;
        lastTimeNs = nowNs;
        if (dt <= 0) return 0.0;

        // If target is zero, reset and coast
        if (targetPos == 0.0) {
            reset();
            return 0.0;
        }

        double error      = targetPos - currentPos;
        double derivative = (error - lastError) / dt;

        // Anti-windup: only integrate when output is not saturated
        if (Math.abs(kP * error) < 1.0) {
            integral = clamp(integral + error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
        }

        // Voltage compensation
        double voltage      = voltageSensor.getVoltage();
        double voltageScale = voltage > 0.1 ? nominalVoltage / voltage : 1.0;

        // Feedforward
        double ff = kF * targetPos * voltageScale;

        double output = kP * error
                + kI * integral
                + kD * derivative
                + ff;

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