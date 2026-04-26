package org.firstinspires.ftc.teamcode.subsystems.constant;

public class IntakeConstants {

    // ── Gate positions ────────────────────────────────────────────────────────
    public static final double closeGatePos = 0.0; // tune
    public static final double openGatePos  = 1.0; // tune

    // ── Motor speeds ──────────────────────────────────────────────────────────
    public static final double intakeSpeed        =  1.0;
    public static final double outtakeSpeed       = -1.0;
    public static final double closeTransferSpeed =  0.5;
    public static final double farTransferSpeed   =  0.8;

    // ── Ball detection thresholds ─────────────────────────────────────────────
    public static final double firstCurrentThreshold  = 3000.0; // mA — tune
    public static final double secondCurrentThreshold = 3000.0; // mA — tune
    public static final double motor2StopThreshold    = 150.0;  // ms — tune
    public static final double motor1StopThreshold    = 150.0;  // ms — tune

    // ── Gate failsafe ─────────────────────────────────────────────────────────
    public static final double gateTime = 500.0; // ms
}
