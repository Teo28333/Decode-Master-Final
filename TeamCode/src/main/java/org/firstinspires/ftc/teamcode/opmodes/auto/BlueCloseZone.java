package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PP Close Zone Blue", group = "Autonomous")
public class BlueCloseZone extends CloseZoneAuto {
    @Override
    protected boolean isBlueAlliance() {
        return true;
    }
}
