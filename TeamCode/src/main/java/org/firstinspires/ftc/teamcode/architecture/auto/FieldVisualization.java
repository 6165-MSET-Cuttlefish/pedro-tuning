package org.firstinspires.ftc.teamcode.architecture.auto;

import static org.firstinspires.ftc.teamcode.core.Robot.robot;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;

public class FieldVisualization {
    public static final double ROBOT_RADIUS = 7.5;

    public static final String COLOR_ROBOT = "#FFFFFF";
    public static final String COLOR_PATH = "#ea8743";
    public static final String COLOR_CURRENT_PATH = "#00ff26";
    public static final String COLOR_HISTORY = "#ff0015";

    private FieldVisualization() {}

    /** Convert a full Pose (x, y, heading) from the original 144×144 system
     *  to the field‑centered system. */
    public static Pose toField(Pose PP) {
        double transX = PP.getX() - 72.0;
        double transY = PP.getY() - 72.0;

        double fieldX = transY;          // original Y → new X
        double fieldY = -transX;         // -original X → new Y

        double fieldTheta = Math.toRadians(
                Math.toDegrees(PP.getHeading()) - 90.0
        );

        return new Pose(fieldX, fieldY, fieldTheta);
    }
    public static double[] toField(double x, double y) {
        double transX = x - 72.0;
        double transY = y - 72.0;
        double fieldX = transY;
        double fieldY = -transX;
        return new double[]{fieldX, fieldY};
    }

    public static void init() {}

    public static void drawRobot(Pose pose) {
        // Dashboard visualization removed
    }

    public static void drawPath(Path path, String color) {
        // Dashboard visualization removed
    }

    public static void drawPath(PathChain pathChain, String color) {
        // Dashboard visualization removed
    }

    public static void drawPoseHistory(PoseHistory poseHistory) {
        // Dashboard visualization removed
    }
}
