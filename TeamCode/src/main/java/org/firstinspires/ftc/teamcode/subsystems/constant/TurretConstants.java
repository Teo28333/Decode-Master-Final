package org.firstinspires.ftc.teamcode.subsystems.constant;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TurretConstants {
    // Hard servo angle limits in degrees. These map to servo positions 0 and 1.
    public static double maxAngleDeg = 160;
    public static double minAngleDeg = -160;
    // If target is outside this safe range, robot auto-turn should help the turret.
    public static double maxSafeAngleDeg = 155;
    public static double minSafeAngleDeg = -155;

    // Turret pivot offset from robot pose in inches.
    public static double TURRET_OFFSET_X = -1.625;
    public static double TURRET_OFFSET_Y = 0.0;

    // Angular velocity smoothing. Higher is smoother but lags more.
    public static double HEADING_VEL_FILTER_ALPHA = 0.8;
    // Predicts robot heading this far ahead to compensate turning while shooting.
    public static double HEADING_VEL_LOOKAHEAD_SEC = 0.375;
    // Linear velocity smoothing for shoot-on-the-move aim lead.
    public static double VELOCITY_FILTER_ALPHA = 0.375;
    // Target angle smoothing. Lower reacts faster; higher reduces twitch.
    public static double TARGET_ANGLE_FILTER_ALPHA = 0.25;
    // Servo command smoothing. Lower moves faster; higher is smoother.
    public static double SERVO_FILTER_ALPHA = 0.05;
    // Ignore tiny servo command changes smaller than this.
    public static double SERVO_DEADBAND = 0.0015;
}
