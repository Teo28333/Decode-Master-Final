package org.firstinspires.ftc.teamcode.subsystems.constant;

public class ShooterConstants {

    // ── PIDF gains — tune in order: kF → kS → kP → kD → kI ──────────────────
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;
    public static double kS = 0.0;

    // ── Controller limits ─────────────────────────────────────────────────────
    public static final double MAX_INTEGRAL   = 0.25;
    public static final double nominalVoltage = 12.0;

    // ── Target threshold ──────────────────────────────────────────────────────
    public static final double RPM_THRESHOLD = 50.0;

    // ── Air time multiplier ───────────────────────────────────────────────────

    public static double airTimeMultiplier = 7.5;

    // ── Tuning mode values ────────────────────────────────────────────────────
    public static double tuningRPM     = 3000.0; // tune
    public static double tuningHoodPos = 0.5;    // tune
}
