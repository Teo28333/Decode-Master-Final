package org.firstinspires.ftc.teamcode.subsystems.constant;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TurretConstants {
    public static double maxAngleDeg = 160;
    public static double minAngleDeg = -160;
    public static double maxSafeAngleDeg = 155;
    public static double minSafeAngleDeg = -155;

    public static double TURRET_OFFSET_X = -1.625;
    public static double TURRET_OFFSET_Y = 0.0;

    public static double HEADING_VEL_FILTER_ALPHA = 0.8;
    public static double HEADING_VEL_LOOKAHEAD_SEC = 0.1;
    public static double VELOCITY_FILTER_ALPHA = 0.8;
}
