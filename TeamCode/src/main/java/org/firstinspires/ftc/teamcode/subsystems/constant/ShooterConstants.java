package org.firstinspires.ftc.teamcode.subsystems.constant;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConstants {
    public static double kP = 0.00825;
    public static double kI = 0.00001;
    public static double kD = 0.0002;
    public static double kF = 0.0001625;
    public static double kS = 0.060;
    public static double MAX_INTEGRAL = 5000;
    public static double RPM_THRESHOLD = 100;
    public static double nominalVoltage = 12.0;
    public static double tuningRPM = 3000.0;
    public static double tuningHoodPos = 0.75;
}
