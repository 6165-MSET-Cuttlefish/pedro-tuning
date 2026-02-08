package org.firstinspires.ftc.teamcode.architecture.vision;

import com.pedropathing.geometry.Pose;

public class AprilTagPose {
    public Pose pose;
    public AllianceColor color;

    public AprilTagPose(Pose pose, AllianceColor color) {
        this.pose = pose;
        this.color = color;
    }

    public Pose getPose() {
        return pose;
    }

    public AllianceColor getColor() {
        return color;
    }

    public String toString() {
        return "Color: " + color.toString() + "\n"
                + "\tPose: " + pose.toString() + "\n"
                + "\t\tX: " + pose.getX() + "\n"
                + "\t\tY: " + pose.getY() + "\n"
                + "\t\tHeading: " + pose.getHeading();
    }
}
