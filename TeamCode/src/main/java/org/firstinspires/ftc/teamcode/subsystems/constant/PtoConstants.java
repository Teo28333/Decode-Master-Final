package org.firstinspires.ftc.teamcode.subsystems.constant;

public class PtoConstants {

    // ── Servo positions ───────────────────────────────────────────────────────
    public static final double engagedPos    = 1.0; // tune
    public static final double disengagedPos = 0.0; // tune

    // ── Lift target (encoder ticks) ───────────────────────────────────────────
    public static final double ticksForLift = 1000.0; // tune

    // ── PIDF ──────────────────────────────────────────────────────────────────
    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;
}
