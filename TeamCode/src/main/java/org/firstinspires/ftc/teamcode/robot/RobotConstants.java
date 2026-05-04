package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConstants {

    // Hardware names from the robot configuration.
    public static final String FRONT_LEFT = "frontLeft";
    public static final String FRONT_RIGHT = "frontRight";
    public static final String BACK_LEFT = "backLeft";
    public static final String BACK_RIGHT = "backRight";

    public static final String INTAKE_MOTOR_1 = "intake1";
    public static final String INTAKE_MOTOR_2 = "intake2";
    public static final String GATE_SERVO = "gate";
    public static final String INTAKE_LED = "led1";

    public static final String SHOOTER_MOTOR_1 = "shooter1";
    public static final String SHOOTER_MOTOR_2 = "shooter2";
    public static final String HOOD_SERVO = "hood";
    public static final String SHOOTER_LED = "led2";

    public static final String TURRET_SERVO_1 = "turret1";
    public static final String TURRET_SERVO_2 = "turret2";

    public static final String LIMELIGHT_NAME = "limelight";

    // Multiplies teleop drive input. Raise for faster driver control.
    public static final double DRIVE_SPEED_MULTIPLIER = 1.25;
    // Auto-turn power used while transferring if the turret is outside safe range.
    public static final double TURN_CORRECTION_POWER = 0.4;
    // Servo position used when the turret fail-safe is enabled.
    public static double TURRET_FAILSAFE_SERVO_POS = 0.5;
    // Max robot turn command while aiming with robot heading in turret fail-safe.
    public static double TURRET_FAILSAFE_MAX_TURN_POWER = 0.55;
    // Heading error allowed before transfer considers fail-safe robot aim ready.
    public static double TURRET_FAILSAFE_AIM_TOLERANCE_RAD = Math.toRadians(2.0);
    // Tune this if the fixed turret fail-safe position is not perfectly aligned with robot heading.
    public static double TURRET_FAILSAFE_HEADING_OFFSET_RAD = 0.0;

    // Limelight relocalization. Keep the filters conservative until the field map is proven.
    public static boolean LIMELIGHT_RELOCALIZATION_ENABLED = true;
    public static boolean LIMELIGHT_APPLY_POSE_UPDATES = false;
    public static boolean LIMELIGHT_USE_MEGATAG2 = true;
    public static int LIMELIGHT_PIPELINE = 0;
    public static int LIMELIGHT_POLL_RATE_HZ = 50;
    public static double LIMELIGHT_RELOCALIZE_INTERVAL_MS = 10000.0;
    public static double LIMELIGHT_MAX_STALENESS_MS = 250.0;
    public static double LIMELIGHT_MAX_POSITION_STDDEV_IN = 18.0;
    public static double LIMELIGHT_MAX_HEADING_STDDEV_DEG = 25.0;
    public static double LIMELIGHT_MAX_CORRECTION_IN = 24.0;
    public static double LIMELIGHT_MAX_RELOCALIZE_SPEED_IN_PER_SEC = 8.0;
    public static double LIMELIGHT_MAX_RELOCALIZE_TURN_DEG_PER_SEC = 30.0;
    public static double LIMELIGHT_X_OFFSET_IN = 0.0;
    public static double LIMELIGHT_Y_OFFSET_IN = 0.0;
    public static double LIMELIGHT_HEADING_OFFSET_RAD = 0.0;

    // Default red start pose if an opmode uses constants instead of saved pose.
    public static final double START_X_RED = 8.25;
    public static final double START_Y_RED = 8.5;
    public static final double START_H_RED = 0.0;

    // Default blue start pose if an opmode uses constants instead of saved pose.
    public static final double START_X_BLUE = 135.75;
    public static final double START_Y_BLUE = 8.5;
    public static final double START_H_BLUE = Math.PI;

    // Turret aim point. Change these if shots are left/right but flywheel speed is good.
    public static double AIM_GOAL_X_RED = 132.0;
    public static double AIM_GOAL_Y_RED = 132.0;
    public static double AIM_GOAL_X_BLUE = 12.0;
    public static double AIM_GOAL_Y_BLUE = 132.0;

    // Flywheel/hood distance point. Change these if RPM/hood distance calculation is off.
    public static double SHOOTING_GOAL_X_RED = 144.0;
    public static double SHOOTING_GOAL_Y_RED = 144.0;
    public static double SHOOTING_GOAL_X_BLUE = 0.0;
    public static double SHOOTING_GOAL_Y_BLUE = 144.0;
}
