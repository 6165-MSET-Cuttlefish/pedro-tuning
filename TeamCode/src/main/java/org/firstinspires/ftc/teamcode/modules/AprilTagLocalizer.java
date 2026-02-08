package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.teamcode.core.Robot.aprilTagTelemetry;
import static org.firstinspires.ftc.teamcode.core.Robot.robot;
import static org.firstinspires.ftc.teamcode.modules.MagazineState.ArtifactColor.EMPTY;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.hardware.Camera;
import org.firstinspires.ftc.teamcode.architecture.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.vision.AprilTagPose;
import org.firstinspires.ftc.teamcode.core.ModuleEx;
import org.firstinspires.ftc.teamcode.lib.State;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class AprilTagLocalizer extends ModuleEx {
    private Camera right;
    private Camera left;
    public static boolean rightCameraRead = true;
    public static boolean leftCameraRead = true;

    public static boolean rightCameraStream = true;
    public static boolean leftCameraStream = false;

    public static double MAX_READ_VEL = 5.0;
    public static double CONVERGENCE_FACTOR = 0.1;

    private Pose followerAprilTagOffset = new Pose(0, 0, 0);

    private ArrayList<AprilTagPose> poses;
    private final ArrayList<PoseHistory> poseHistory = new ArrayList<>();
    private Pose avgPose;
    private final ElapsedTime timer = new ElapsedTime();

    public enum LocalizerState implements State {
        ON(1),
        OFF(0);

        LocalizerState(double value) {
            setValue(value);
        }
    }

    public AprilTagLocalizer(HardwareMap hardwareMap) {
        super();
        setTelemetryEnabled(aprilTagTelemetry.TOGGLE);

        int monitorId1 = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int monitorId2 = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId2", "id", hardwareMap.appContext.getPackageName());

        if (rightCameraRead) {
            right = new Camera(hardwareMap, "right", new Pose(7.5, -8.5, -30),
                    new AprilTagProcessor.Builder()
                            .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                            .setLensIntrinsics(919.896, 919.896, 601.543, 309.905)
                            .build(),
                    monitorId1, rightCameraStream);
        }

        if (leftCameraRead) {
            left = new Camera(hardwareMap, "left", new Pose(7.5, 8.5, 30),
                    new AprilTagProcessor.Builder()
                            .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                            .setLensIntrinsics(919.896, 919.896, 601.543, 309.905)
                            .build(),
                    monitorId2, leftCameraStream);
        }
    }

    @Override
    protected void initStates() {
        setStates(LocalizerState.OFF);
    }

    @Override
    protected void read() {
        if (rightCameraRead) {
            right.update();
        }
        if (leftCameraRead) {
            left.update();
        }

        poses = new ArrayList<>();
        if (rightCameraRead && !right.poses.isEmpty()) {
            poses.addAll(right.poses);
        }
        if (leftCameraRead && !left.poses.isEmpty()) {
            poses.addAll(left.poses);
        }
        drawRobotOnField();

        updatePoseHistory();
        updateCameraStreamActive();
    }

    @Override
    protected void write() {}

    private void updateCameraStreamActive() {
        right.updateCameraStream(rightCameraStream);
        left.updateCameraStream(leftCameraStream);
    }

    private void drawRobotOnField() {
        // Dashboard visualization removed - calculations skipped
    }

    public Pose addHistoryOffset(Pose pose) {
        return pose.plus(getDeltaPose(500));
    }

    private void updatePoseHistory() {
        poseHistory.add(new PoseHistory());
        double dt = timer.milliseconds();
        for (PoseHistory history : poseHistory) {
            history.updateTime(dt);
        }
        timer.reset();
    }

    private Pose getDeltaPose(double aprilTagLoopTime) {
        Pose current = robot.follower.getPose();
        if (poseHistory.isEmpty())
            return current.minus(current);

        PoseHistory closest = poseHistory.get(0);
        double bestDiff = Math.abs(closest.timeInPast - aprilTagLoopTime);

        for (PoseHistory h : poseHistory) {
            double diff = Math.abs(h.timeInPast - aprilTagLoopTime);
            if (diff < bestDiff) {
                bestDiff = diff;
                closest = h;
            }
        }

        Pose delta = current.minus(closest.getPose());

        PoseHistory finalClosest = closest;
        poseHistory.removeIf(h -> h.timeInPast > aprilTagLoopTime && !h.equals(finalClosest));

        return delta;
    }

    public Pose getPose() {
        return avgPose;
    }

    public void stopCameraStreams() {
        right.visionPortal.close();
        left.visionPortal.close();
    }

    public MagazineState getObelisk() {
        if (rightCameraRead) {
            return right.getObelisk();
        }
        if (leftCameraRead) {
            return left.getObelisk();
        }
        return new MagazineState(EMPTY, EMPTY, EMPTY);
    }

    public Pose getConvergencePose() {
        if (robot.follower.getVelocity().getMagnitude() < MAX_READ_VEL) {
            Pose targetOffset =
                    avgPose.minus(robot.follower.getPose().plus(followerAprilTagOffset));
            followerAprilTagOffset = followerAprilTagOffset.times(1 - CONVERGENCE_FACTOR)
                                             .plus(targetOffset.times(CONVERGENCE_FACTOR));
        }
        return robot.follower.getPose().plus(followerAprilTagOffset);
    }

    public Pose getCameraPose() {
        return new Pose(avgPose.getX() + 72, avgPose.getY() + 72, avgPose.getHeading());
    }

    public void closeAll() {
        if (rightCameraRead) {
            right.close();
        }
        if (leftCameraRead) {
            left.close();
        }
    }

    @Override
    protected void onTelemetry() {
        if (aprilTagTelemetry.TOGGLE) {
            logDashboard("April Tag Localizer", poses.size());

            if (rightCameraRead) {
            }
            if (leftCameraRead) {
            }
            logDashboard("RobotPos: ", robot.follower.getPose().toString());
            logDashboard("Follower-AprilTag Offset", followerAprilTagOffset.toString());
        }
    }
}

class PoseHistory {
    double timeInPast;
    Pose pose;

    public PoseHistory() {
        timeInPast = 0;
        pose = robot.follower.getPose();
    }

    public void updateTime(double time) {
        timeInPast += time;
    }

    public Pose getPose() {
        return pose;
    }
}
