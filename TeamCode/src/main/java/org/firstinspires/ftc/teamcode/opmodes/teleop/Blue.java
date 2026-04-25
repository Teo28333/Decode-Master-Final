package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
public class Blue extends OpMode {

    private Robot robot;
    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, true, false);
    }
    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        telemetry.update();
        robot.update(gamepad1);
    }
}
