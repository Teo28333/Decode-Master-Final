package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;

public class PoseStorage {
    public static Pose currentPose = new Pose(72, 72, 0);

    public static void setCurrentPose(Pose pose) {
        if (pose != null) {
            currentPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());
        }
    }
}
