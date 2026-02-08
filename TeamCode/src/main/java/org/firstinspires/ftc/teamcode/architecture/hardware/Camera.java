package org.firstinspires.ftc.teamcode.architecture.hardware;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.architecture.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.vision.AprilTagPose;
import org.firstinspires.ftc.teamcode.core.Context;
import org.firstinspires.ftc.teamcode.modules.MagazineState;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.core.Robot.aprilTagTelemetry;
import static org.firstinspires.ftc.teamcode.core.Robot.robot;
import static org.firstinspires.ftc.teamcode.modules.MagazineState.ArtifactColor.*;

@Config
public class Camera {
    private final AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    private final int containerId;
    private final String cameraName;

    public static float decimation = 1;

    private final Pose cameraPose;
    private List<AprilTagDetection> detections;

    public ArrayList<AprilTagPose> poses = new ArrayList<>();

    private boolean isStreaming = false;

    public Camera(HardwareMap hardwareMap, String cameraName, Pose cameraPose,
            AprilTagProcessor aprilTagProcessor, int containerId, boolean cameraStream) {
        this.cameraPose = cameraPose;
        this.aprilTag = aprilTagProcessor;
        this.containerId = containerId;
        this.cameraName = cameraName;

        aprilTag.setDecimation(decimation);

        VisionPortal.Builder builder =
                new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                        .setCameraResolution(new Size(1280, 720))
                        .setLiveViewContainerId(containerId)
                        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                        .setAutoStopLiveView(false)
                        .addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    public void update() {
        poses = calculatePoses();
    }

    public ArrayList<AprilTagPose> calculatePoses() {
        ArrayList<AprilTagPose> poses = new ArrayList<>();
        detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 20 || detection.id == 24) {
                poses.add(calculateRobotPos(detection, cameraPose));
            }
        }
        return poses;
    }

    public MagazineState getObelisk() {
        detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            switch (detection.id) {
                case 21:
                    return new MagazineState(GREEN, PURPLE, PURPLE);
                case 22:
                    return new MagazineState(PURPLE, GREEN, PURPLE);
                case 23:
                    return new MagazineState(PURPLE, PURPLE, GREEN);
            }
        }
        return new MagazineState(EMPTY, EMPTY, EMPTY);
    }

    public void updateCameraStream(boolean command) {
        if (isStreaming && !command) {
            FtcDashboard.getInstance().stopCameraStream();
            isStreaming = false;
        } else if (!isStreaming && command) {
            FtcDashboard.getInstance().startCameraStream(visionPortal, 10);
            isStreaming = true;
        }
    }

    public void close() {
        visionPortal.close();
    }

    private AprilTagPose calculateRobotPos(AprilTagDetection detection, Pose cameraPose) {
        String tag = detection.id == 24 ? "red: " : "blue: ";
        robot.telemetry.addDashboardData(tag + "Using apriltag ID:" + detection.id, "");

        double tagX = 0;
        double tagY = 0;
        double tagH = 54;

        AllianceColor color = null;
        if (detection.id == 24) {
            tagX = Context.redApriltagPose.getX();
            tagY = Context.redApriltagPose.getY();
            color = AllianceColor.RED;
        } else if (detection.id == 20) {
            tagX = Context.blueApriltagPose.getX();
            tagY = Context.blueApriltagPose.getY();
            color = AllianceColor.BLUE;
        }

        double bearing = detection.ftcPose.bearing;
        double yaw = detection.ftcPose.yaw;
        double pitch = detection.ftcPose.pitch;
        double roll = detection.ftcPose.roll;

        double vx = detection.ftcPose.y * Math.cos(Math.toRadians(pitch));
        double vy = detection.ftcPose.x;

        double lateralRange = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));

        robot.telemetry.addDashboardData(tag + "lateral range: ", lateralRange);
        robot.telemetry.addDashboardData(tag + "pitch: ", pitch);
        robot.telemetry.addDashboardData(tag + "bearing: ", bearing);
        robot.telemetry.addDashboardData(tag + "yaw: ", yaw);
        robot.telemetry.addDashboardData(tag + "roll: ", roll);

        double difytagh = 0;
        double theta = 0;
        difytagh = 0;

        if (detection.id == 24) {
            difytagh = tagH + yaw;
            theta = (90 + (bearing)) - difytagh;
        } else {
            difytagh = tagH - yaw;
            theta = (90 - (bearing)) - difytagh;
        }

        robot.telemetry.addDashboardData(tag + "tagH - yaw = ", difytagh);
        robot.telemetry.addDashboardData(tag + "theta: ", theta);

        double x = lateralRange * Math.cos(Math.toRadians(theta));
        double y = lateralRange * Math.sin(Math.toRadians(theta));

        double fieldRobotX = tagX + y;
        double fieldRobotY = detection.id == 24 ? tagY - x : tagY + x;

        robot.telemetry.addDashboardData(tag + "robotx: ", x);
        robot.telemetry.addDashboardData(tag + "roboty: ", y);

        double heading = 0;
        if (detection.id == 24) {
            heading = 90 + theta - bearing;
        } else {
            heading = -90 - theta - bearing;
        }

        robot.telemetry.addDashboardData("field heading: ", heading);

        Pose camPose = new Pose(fieldRobotX, fieldRobotY, heading);
        Pose includesCamOffset = cameraFieldToRobotField(camPose, cameraPose);

        robot.telemetry.addDashboardData(tag + "fieldRobotX: ", includesCamOffset.getX());
        robot.telemetry.addDashboardData(tag + "fieldRobotY: ", includesCamOffset.getY());

        return new AprilTagPose(includesCamOffset, color);
    }

    public Pose cameraFieldToRobotField(Pose camPoseField, Pose cameraPose) {
        double xC = camPoseField.getX();
        double yC = camPoseField.getY();
        double thC = camPoseField.getHeading();

        double th = Math.toRadians(thC);
        double xOff = cameraPose.getX();
        double yOff = cameraPose.getY();
        double hOff = cameraPose.getHeading();

        double dx = xOff * Math.cos(th) - yOff * Math.sin(th);
        double dy = xOff * Math.sin(th) + yOff * Math.cos(th);

        double xR = xC - dx;
        double yR = yC - dy;

        double robotHeading = normalizeDegrees(thC - hOff);

        return new Pose(xR, yR, robotHeading);
    }

    private double normalizeDegrees(double angle) {
        double a = angle % 360.0;
        if (a < -180.0)
            a += 360.0;
        if (a > 180.0)
            a -= 360.0;
        return a;
    }

    public void telemetryUpdate() {
        String telemetryString = "\t" + cameraName + "\n"
                + "\t"
                + "Dashboard Stream: " + isStreaming + "\n";
        if (aprilTagTelemetry.raw) {
            for (AprilTagDetection detection : detections) {
                telemetryString += "\t\t"
                        + "Tag ID" + detection.id + "\n"
                        + "\t\t\t"
                        + "Range" + detection.ftcPose.range + "\n"
                        + "\t\t\t"
                        + "X" + detection.ftcPose.x + "\n"
                        + "\t\t\t"
                        + "Y" + detection.ftcPose.y + "\n"
                        + "\t\t\t"
                        + "Yaw" + detection.ftcPose.yaw + "\n";
                telemetryString += "\t"
                        + "ApriltagPoses: " + poses.size() + "\n";
                for (int i = 0; i < poses.size(); i++) {
                    telemetryString += "\t\t"
                            + "AprilTagPose " + i + ": " + poses.get(i).toString() + "\n";
                }
            }
        }
        robot.telemetry.addDashboardData("Camera: ", telemetryString);
    }
}
