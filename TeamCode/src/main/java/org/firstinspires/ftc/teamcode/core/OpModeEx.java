package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.core.Robot.telemetryToggles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.lib.EnhancedOpMode;

public abstract class OpModeEx extends EnhancedOpMode {
    protected Robot robot;

    @Override
    protected final void setup() {
        try {
            robot = new Robot(this, shouldPreservePosition());
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry = robot.telemetry;

        robot.packet = new TelemetryPacket(false);

        register(
                robot.drivetrain, robot.shooter, robot.magazine, robot.turret, robot.srsHubManager, robot.endgame);
    }

    @Override
    protected final void onInit() {
        initialize();
    }

    @Override
    protected final void onInitLoop() {
        robot.telemetry.addData("Alliance Color", Context.allianceColor);
        Pose currentPose = robot.follower.getPose();
        robot.telemetry.addData("Robot Position", "X: %.1f, Y: %.1f, Heading: %.1f°", currentPose.getX(),
                currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        if (telemetryToggles.voltage) {
            telemetry.addData("🔋 Voltage", "%.2fV", getVoltage());
        }
        if (telemetryToggles.current) {
            telemetry.addData("\uD83C\uDF0A Current", "%.2fA", getTotalCurrent());
        }
        robot.follower.update();
        initializeLoop();
        updateDashboard();
    }

    @Override
    protected final void onLoop() {
        robot.telemetry.addData("Alliance Color", Context.allianceColor);
        Pose currentPose = robot.follower.getPose();
        robot.telemetry.addData("Robot Position", "X: %.1f, Y: %.1f, Heading: %.1f°", currentPose.getX(),
                currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        if (telemetryToggles.voltage) {
            telemetry.addData("🔋 Voltage", "%.2fV", getVoltage());
        }
        if (telemetryToggles.current) {
            telemetry.addData("\uD83C\uDF0A Current", "%.2fA", getTotalCurrent());
        }
        robot.updateWriteToggles();
        robot.follower.update();
        primaryLoop();
        updateDashboard();
    }

    @Override
    protected final void onStop() {
        onEnd();
    }

    protected void initialize() {}

    protected void initializeLoop() {}

    protected void primaryLoop() {}

    protected void onEnd() {}

    protected boolean shouldPreservePosition() {
        return true;
    }

    protected void updateDashboard() {
        Canvas overlay = robot.packet.fieldOverlay();

        overlay.setAlpha(0.4);
        overlay.drawImage("/images/fieldcoordinates-pedro.webp", 0, 0, 144, 144);
        overlay.setAlpha(1);
        overlay.drawGrid(0, 0, 144, 144, 7, 7);

        FieldVisualization.drawRobot(robot.follower.getPose());

        FtcDashboard.getInstance().sendTelemetryPacket(robot.packet);
        robot.packet = new TelemetryPacket(false);
    }
}
