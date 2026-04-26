package org.firstinspires.ftc.teamcode.subsystems.constant;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class IntakeConstants {
    public static double intakeSpeed = 1.0;
    public static double farTransferSpeed = 1.0;
    public static double closeTransferSpeed = 1.0;
    public static double outtakeSpeed = -0.75;

    public static double motor2StopThreshold = 250;
    public static double motor1StopThreshold = 1250;
    public static double firstCurrentThreshold = 2500;
    public static double secondCurrentThreshold = 2700;
    public static double gateTime = 500;

    public static double openGatePos = 0.5;
    public static double closeGatePos = 0.1;
}
