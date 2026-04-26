package org.firstinspires.ftc.teamcode.subsystems.constant;

public class TurretConstants {

    // ── Servo range (degrees) ─────────────────────────────────────────────────
    public static final double minAngleDeg     = -160.0;
    public static final double maxAngleDeg     =  160.0;
    public static final double minSafeAngleDeg = -150.0;
    public static final double maxSafeAngleDeg =  150.0;

    // ── Physical mount offset (inches from robot center to turret pivot) ───────
    // TURRET_OFFSET_X: positive = turret is in front of robot center
    // TURRET_OFFSET_Y: positive = turret is to the left of robot center
    public static double TURRET_OFFSET_X = 0.0;
    public static double TURRET_OFFSET_Y = 0.0;

    // ── Velocity filters ──────────────────────────────────────────────────────
    // Alpha values: 0 = no filter (raw), 1 = frozen. Start at 0.8 and tune.
    public static double VELOCITY_FILTER_ALPHA     = 0.8;
    public static double HEADING_VEL_FILTER_ALPHA  = 0.8;

    // Lookahead in seconds — lead heading to compensate for servo latency
    // Start at 0.0, increase in 0.05s steps until turret tracks well during spin
    public static double HEADING_VEL_LOOKAHEAD_SEC = 0.1;
}
