package org.firstinspires.ftc.teamcode.robot;

public class RobotConstants {

    // ── Drivetrain ────────────────────────────────────────────────────────────
    public static final String FRONT_LEFT  = "frontLeft";
    public static final String FRONT_RIGHT = "frontRight";
    public static final String BACK_LEFT   = "backLeft";
    public static final String BACK_RIGHT  = "backRight";

    // ── Intake ────────────────────────────────────────────────────────────────
    public static final String INTAKE_MOTOR_1 = "intake1";
    public static final String INTAKE_MOTOR_2 = "intake2";
    public static final String GATE_SERVO     = "gate";
    public static final String INTAKE_LED     = "led1";

    // ── Shooter ───────────────────────────────────────────────────────────────
    public static final String SHOOTER_MOTOR_1 = "shooter1";
    public static final String SHOOTER_MOTOR_2 = "shooter2";
    public static final String HOOD_SERVO      = "hood";
    public static final String SHOOTER_LED     = "led2";

    // ── Turret ────────────────────────────────────────────────────────────────
    public static final String TURRET_SERVO_1 = "turret1";
    public static final String TURRET_SERVO_2 = "turret2";

    // ── PTO ───────────────────────────────────────────────────────────────────
    public static final String PTO_SERVO = "pto";

    // ── Drive tuning ──────────────────────────────────────────────────────────
    public static final double DRIVE_SPEED_MULTIPLIER = 1.0; // reduce to slow robot
    public static final double TURN_CORRECTION_POWER  = 0.4; // auto-rotate during transfer

    // ── Starting poses ────────────────────────────────────────────────────────
    public static final double START_X_RED  = 8.25;
    public static final double START_Y_RED  = 8.5;
    public static final double START_H_RED  = 0.0;

    public static final double START_X_BLUE = 135.75;
    public static final double START_Y_BLUE = 8.5;
    public static final double START_H_BLUE = Math.PI;

    // ── Goal positions ────────────────────────────────────────────────────────
    public static final double GOAL_X_RED  = 142.0;
    public static final double GOAL_Y_RED  = 144.0;
    public static final double GOAL_X_BLUE = 2.0;
    public static final double GOAL_Y_BLUE = 144.0;

    // ── Headings ──────────────────────────────────────────────────────────────
    public static final double HEADING_RED  = 0.0;
    public static final double HEADING_BLUE = Math.PI;
}
