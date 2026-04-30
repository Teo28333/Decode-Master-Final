package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PP Solo Blue", group = "Autonomous")
public class BlueSolo extends SoloAuto {
    @Override
    protected boolean isBlueAlliance() {
        return true;
    }
}
