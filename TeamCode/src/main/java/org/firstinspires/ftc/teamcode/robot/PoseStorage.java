package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;

public class PoseStorage {
    private static final double FIELD_SIZE = 144.0;

    public static Pose currentPose = new Pose(72, 72, 0);

    public static void setCurrentPose(Pose pose) {
        if (pose != null) {
            currentPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());
        }
    }

    public static boolean hasValidPose() {
        return isValid(currentPose);
    }

    public static boolean isValid(Pose pose) {
        return pose != null
                && !Double.isNaN(pose.getX())
                && !Double.isNaN(pose.getY())
                && !Double.isNaN(pose.getHeading())
                && pose.getX() >= 0.0
                && pose.getX() <= FIELD_SIZE
                && pose.getY() >= 0.0
                && pose.getY() <= FIELD_SIZE;
    }

    public static Pose allianceStartPose(boolean isBlueAlliance) {
        if (isBlueAlliance) {
            return new Pose(RobotConstants.START_X_BLUE, RobotConstants.START_Y_BLUE, RobotConstants.START_H_BLUE);
        }

        return new Pose(RobotConstants.START_X_RED, RobotConstants.START_Y_RED, RobotConstants.START_H_RED);
    }

    public static Pose fieldCenterPose() {
        return new Pose(72.0, 72.0, 0.0);
    }

}
