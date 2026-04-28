package org.firstinspires.ftc.teamcode.subsystems.constant;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class PtoConstants {
    // PTO servo position when drivetrain is normal driving mode.
    public static double disengagedPos = 0.0;
    // PTO servo position when lift/PTO is engaged.
    public static double engagedPos = 0.5;
    // Maximum lift motor power.
    public static double liftMaxSpeed = 1.0;
    // Minimum lift motor power when commanding lift.
    public static double liftMinSpeed = 0.0;
    // Encoder target for lift travel.
    public static double ticksForLift = 0.0;

    // Lift PID proportional gain. Raise if it is too weak.
    public static double kP = 0.0;
    // Lift PID integral gain. Usually keep near zero.
    public static double kI = 0.0;
    // Lift PID derivative gain. Adds damping if it overshoots.
    public static double kD = 0.0;
    // Lift feedforward gain.
    public static double kF = 0.0;
    // Voltage reference for lift feedforward compensation.
    public static double nominalVoltage = 0.0;

}
