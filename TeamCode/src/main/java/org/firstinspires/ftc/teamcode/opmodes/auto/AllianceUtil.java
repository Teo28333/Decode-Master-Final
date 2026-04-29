package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.Pose;

final class AllianceUtil {
    static final double FIELD_SIZE = 144.0;

    private AllianceUtil() {
    }

    static Pose mirrorForAlliance(Pose bluePose, boolean isBlueAlliance) {
        if (isBlueAlliance) {
            return bluePose;
        }

        return new Pose(FIELD_SIZE - bluePose.getX(), bluePose.getY(), Math.PI - bluePose.getHeading());
    }
}
