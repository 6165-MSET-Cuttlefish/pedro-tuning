package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;
import com.skeletonarmy.marrow.prompts.MultiOptionPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.ValuePrompt;
import org.firstinspires.ftc.teamcode.architecture.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.core.Context;
import org.firstinspires.ftc.teamcode.core.OpModeEx;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.Magazine;

import static org.firstinspires.ftc.teamcode.core.Context.pinpointPose;

public abstract class Auto extends OpModeEx {
    protected Pose startPose;
    protected PoseHistory poseHistory;
    protected Timer actionTimer;
    protected Prompter prompter = new Prompter(this);
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

        prompter.prompt("color", new OptionPrompt<>("Select Color", AllianceColor.RED, AllianceColor.BLUE))
                .prompt("delayMs", new ValuePrompt("Start Delay (ms)", 0, 10000, 0, 1000))
                .prompt("far/close", new OptionPrompt<>("Select Mode", "Far", "Close"))
                .prompt("path segments", () -> {
                    if (prompter.get("far/close").equals("Far")) {
                        return new MultiOptionPrompt<>(
                                "Select Far Path Segments",
                                false,
                                true,
                                3,
                                "runIntakeScore1Choice", "runIntakeScore2Choice", "runGateChoice"); //might have to be enum with T/F?
                    }
                    return new MultiOptionPrompt<>(
                            "Select Close Path Segments",
                            false,
                            true,
                            4,
                            "runIntakeScore1Choice", "runIntakeScore2Choice", "runGateChoice", "runIntakeGateChoice");
                });

        prompter.onComplete(this::onPromptsComplete);

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
        prompter.run();

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
    protected void telemetry() {
        robot.telemetry.addDashboardData("Time Elapsed", "%.1f seconds", actionTimer.getElapsedTimeSeconds());
        robot.telemetry.addDashboardData("Scheduler", robot.pathActionScheduler.getDebugInfo());
        robot.telemetry.addDashboardData("Total Segments", robot.pathActionScheduler.getTotalSegments());
        robot.telemetry.addDashboardData("Obelisk", Context.motif.toString());
        robot.telemetry.addDashboardData("delayMs", delayMs);
    }
    protected void onPromptsComplete() {
            Context.allianceColor = prompter.get("color");
            delayMs = prompter.get("delayMs");
//            String pathSegments = prompter.get("path segments"); //not exactly sure what type MultiOptionPrompt returns so will test
//            RobotLog.e("pathSegments: " + pathSegments + "..." + prompter.get("path segments").toString());

        // set the variables in close/far, like runIntakeScore1 = runIntakeScore1Choice
//            if (prompter.get("far/close").equals("Far")) {
//                Far.runIntakeScore1 = (boolean) prompter.get("path segments"); //get runIntakeScore1Choice
//                //other far segments
//            } else {
//                Close.runIntakeScore1 = (boolean) prompter.get("path segments");
//                //other close segments
//            }
    }
}
