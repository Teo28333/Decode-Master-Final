package org.firstinspires.ftc.teamcode.subsystems.constant;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class IntakeConstants {
    // Power used while collecting balls. Usually keep at full power unless jams increase.
    public static double intakeSpeed = 1.0;
    // Transfer power from far/back shooting zone. Lower if balls feed too violently.
    public static double farTransferSpeed = 1.0;
    // Transfer power from close/front shooting zone.
    public static double closeTransferSpeed = 1.0;
    // Reverse power for clearing balls.
    public static double outtakeSpeed = -0.75;

    // Time motor 2 current must stay high before the first ball is considered captured.
    public static double motor2StopThreshold = 250;
    // Time motor 1 current must stay high before the second ball is considered captured.
    public static double motor1StopThreshold = 750;
    // Current threshold for the first ball detection stage.
    public static double firstCurrentThreshold = 2500;
    // Current threshold for the second ball detection stage.
    public static double secondCurrentThreshold = 2600;
    // Delay after gate movement before motors can run.
    public static double gateSettleTime = 250;
    // Current on both motors above this may mean transfer is jammed.
    public static double transferJamCurrentThreshold = 3700;
    // How long jam current must persist before transfer latches JAMMED.
    public static double transferJamTime = 750;

    // Servo position for gate open during transfer/outtake.
    public static double openGatePos = 0.5;
    // Servo position for gate closed during intake.
    public static double closeGatePos = 0.1;
}
