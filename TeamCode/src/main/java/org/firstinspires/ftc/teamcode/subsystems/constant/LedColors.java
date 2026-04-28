package org.firstinspires.ftc.teamcode.subsystems.constant;

public class LedColors {

    // Intake LED servo positions.
    // Idle/no confirmed ball.
    public static final double INTAKE_IDLE = 0.28;
    // First ball detected at the rear/second motor.
    public static final double INTAKE_BALL_MOTOR2 = 0.65;
    // Full intake/second ball detected.
    public static final double INTAKE_BALL_MOTOR1 = 0.50;

    // Shooter LED servo positions.
    // Shooter is ready/on target.
    public static final double SHOOTER_AT_TARGET = 0.60;
    // Shooter is below target or recovering.
    public static final double SHOOTER_SPINNING_UP = 0.30;
}
