package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class AutoConstants {
    public static double CLOSE_PRELOAD_SHOOT_MS = 1000.0;
    public static double CLOSE_SHOOT_MS = 1000.0;
    public static double CLOSE_GATE_CONFIRM_INTAKE_MS = 1000.0;
    public static double CLOSE_RETURN_INTAKE_MS = 250.0;
    public static int CLOSE_MIN_GATE_INTAKES = 1;
    public static int CLOSE_MAX_GATE_INTAKES = 6;
    public static int CLOSE_DEFAULT_GATE_INTAKES = 4;

    public static double FAR_PRELOAD_SPIN_UP_MS = 2250.0;
    public static double FAR_PRELOAD_SHOOT_MS = 1000.0;
    public static double FAR_SHOOT_MS = 1000.0;
    public static double FAR_HUMAN_PLAYER_CONFIRM_INTAKE_MS = 1000.0;
    public static double FAR_RETURN_INTAKE_MS = 250.0;

    public static double PATH_STEP_TIMEOUT_MS = 4500.0;
    public static double SHOOTER_WAIT_TIMEOUT_MS = 2500.0;
    public static double SOLO_PATH_STEP_TIMEOUT_MS = 4500.0;
    public static double SOLO_PRELOAD_SPIN_UP_MS = 2250.0;
    public static double SOLO_PRELOAD_SHOOT_MS = 1000.0;
    public static double SOLO_SHOOT_MS = 1000.0;
    public static double SOLO_HUMAN_PLAYER_CONFIRM_INTAKE_MS = 1000.0;
    public static double SOLO_RETURN_INTAKE_MS = 250.0;
}
