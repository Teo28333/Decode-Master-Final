package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PP Move Only Red", group = "Autonomous")
public class RedFar extends FarAuto {
    @Override
    protected boolean isBlueAlliance() {
        return false;
    }
}
