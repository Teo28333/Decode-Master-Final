package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PP Close Zone Red", group = "Autonomous")
public class RedCloseZone extends CloseZoneAuto {
    @Override
    protected boolean isBlueAlliance() {
        return false;
    }
}
