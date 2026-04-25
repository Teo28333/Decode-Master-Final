package org.firstinspires.ftc.teamcode.opmodes.tuning;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp
@Config
public class TurretTesting extends OpMode {

    private ServoImplEx ts1, ts2;
    public static double turretAngle = 0.0;
    public static double minPose = -160;
    public static double maxPose = 160;

    @Override
    public void init() {
        ts1 = hardwareMap.get(ServoImplEx.class, "turret1");
        ts2 = hardwareMap.get(ServoImplEx.class, "turret2");

        PwmControl.PwmRange fullRange = new PwmControl.PwmRange(500, 2500);
        ts1.setPwmRange(fullRange);
        ts2.setPwmRange(fullRange);
        ts1.setPosition(0.5);
        ts2.setPosition(0.5);
    }

    @Override
    public void loop() {
        ts1.setPosition(angleDegToServoPos(turretAngle));
        ts2.setPosition(angleDegToServoPos(turretAngle));
    }

    private double angleDegToServoPos(double angleDeg) {
        double pos = (angleDeg - minPose) / (maxPose - minPose);
        return Range.clip(pos, 0.0, 1.0);
    }
}