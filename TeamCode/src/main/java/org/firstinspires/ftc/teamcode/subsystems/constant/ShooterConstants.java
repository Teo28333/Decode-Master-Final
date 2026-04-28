package org.firstinspires.ftc.teamcode.subsystems.constant;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConstants {
    // Main velocity PID. Raise kP if spin-up/recovery is lazy; lower it if RPM oscillates.
    public static double kP = 0.0065;
    // Integral removes steady-state RPM error. Keep small; too high causes windup.
    public static double kI = 0.00001;
    // Derivative damps overshoot. Raise slightly if the shooter spikes past target.
    public static double kD = 0.00005;
    // Feedforward power per RPM. Tune first so steady RPM holds with little PID work.
    public static double kF = 0.0001625;
    // Static friction/start power added with feedforward.
    public static double kS = 0.060;
    // Max integral accumulator. Lower if the controller stays wound up.
    public static double MAX_INTEGRAL = 5000;

    // Shooter is ready inside this RPM window around target.
    public static double RPM_THRESHOLD = 150;
    // RPM loss before transfer recovery starts adding extra commanded speed.
    public static double TRANSFER_RECOVERY_DEADBAND_RPM = 50;
    // Extra RPM command per RPM lost past the deadband while the transfer is active.
    public static double TRANSFER_RECOVERY_MULTIPLIER = 2.0;
    // Hood drops by this much at or above MAX_HOOD_DROP_RPM_LOSS.
    public static double MAX_HOOD_RECOVERY_DROP = 0.075;
    // RPM loss where the hood recovery drop reaches its maximum.
    public static double MAX_HOOD_DROP_RPM_LOSS = 250;
    // Voltage used as the feedforward reference.
    public static double nominalVoltage = 12.0;

    // Manual shooter tuning mode values.
    public static double tuningRPM = 3000.0;
    public static double tuningHoodPos = 0.75;
}
