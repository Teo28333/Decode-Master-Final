package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PP Solo Red", group = "Autonomous")
public class RedSolo extends SoloAuto {
    @Override
    protected boolean isBlueAlliance() {
        return false;
    }
}
