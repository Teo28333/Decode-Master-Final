package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PP Move Only Blue", group = "Autonomous")
public class BlueFar extends FarAuto {
    @Override
    protected boolean isBlueAlliance() {
        return true;
    }
}
