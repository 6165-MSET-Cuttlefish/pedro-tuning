package org.firstinspires.ftc.teamcode.architecture.auto;

import static org.firstinspires.ftc.teamcode.core.Context.pinpointPose;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.architecture.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.core.Context;
import org.firstinspires.ftc.teamcode.core.OpModeEx;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.Magazine;

public abstract class Auto extends OpModeEx {
    protected Pose startPose;
    protected PoseHistory poseHistory;
    protected Timer actionTimer;
    protected int delayMs = 0; //offset auto for alliance
    protected boolean runIntakeScore1Choice = true, runIntakeScore2Choice = true,
           runGateChoice = true, runIntakeGateChoice = true;

    private void renderVisualization(boolean drawPlannedPaths) {
        if (drawPlannedPaths) {
            PathChain[] paths = getPathsForVisualization();
            for (PathChain path : paths) {
                FieldVisualization.drawPath(path, FieldVisualization.COLOR_PATH);
            }
        }

        if (robot.pathActionScheduler.getCurrentPath() != null) {
            FieldVisualization.drawPath(robot.pathActionScheduler.getCurrentPath(),
                    FieldVisualization.COLOR_CURRENT_PATH);
        }

        FieldVisualization.drawPoseHistory(poseHistory);
    }

    @Override
    protected boolean shouldWriteDuringInit() {
        return true;
    }

    @Override
    protected boolean shouldPreservePosition() {
        return false;
    }

    protected abstract Pose getSetupPose();

    protected abstract void buildAutonomousSequence();

    protected PathChain[] getPathsForVisualization() {
        return new PathChain[0];
    }

    @Override
    protected void initialize() {
        startPose = getSetupPose();
        robot.follower.setPose(startPose);

        Drivetrain.DriveState.EXTERNAL.apply();

        poseHistory = robot.follower.getPoseHistory();
        FieldVisualization.init();
        actionTimer = new Timer();

        Magazine.HeadlightState.STROBE.apply();

    }

    @Override
    protected void initializeLoop() {
        buildAutonomousSequence();

        if (Context.allianceColor.equals(AllianceColor.RED)) {
            Context.motif = robot.leftCamera.getObelisk();
            robot.leftCamera.updateCameraStream(Context.leftCameraStream);
        } else {
            Context.motif = robot.rightCamera.getObelisk();
            robot.rightCamera.updateCameraStream(Context.rightCameraStream);
        }

        renderVisualization(true);
    }

    @Override
    protected void onStart() {
        actionTimer.resetTimer();
        renderVisualization(true);
    }

    @Override
    protected void primaryLoop() {
        if (robot.drivetrain.isBonk()) {
            robot.pathActionScheduler.skipCurrentSegment();
            robot.drivetrain.bonkTimer.reset();
        }
        robot.pathActionScheduler.update();

        pinpointPose = robot.follower.getPose();
        robot.leftCamera.updateCameraStream(Context.leftCameraStream);
        robot.rightCamera.updateCameraStream(Context.rightCameraStream);

        renderVisualization(true);
    }

    @Override
    protected void telemetry() {}
}
