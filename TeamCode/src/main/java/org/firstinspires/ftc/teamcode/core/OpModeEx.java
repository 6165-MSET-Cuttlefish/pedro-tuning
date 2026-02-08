package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.lib.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.lib.EnhancedTelemetry;

import static org.firstinspires.ftc.teamcode.core.Robot.telemetryToggles;

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
        robot.telemetry.addGroupHeader("ROBOT STATUS");
        addAllianceTelemetry();
        Pose currentPose = robot.follower.getPose();
        robot.telemetry.addData("Robot Position", "X: %.1f, Y: %.1f, Heading: %.1f°", currentPose.getX(),
                currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        addVoltageCurrentTelemetry();
        robot.follower.update();
        initializeLoop();
        updateDashboard();
    }

    @Override
    protected final void onLoop() {
        robot.telemetry.addGroupHeader("ROBOT STATUS");
        addAllianceTelemetry();
        Pose currentPose = robot.follower.getPose();
        robot.telemetry.addData("Robot Position", "X: %.1f, Y: %.1f, Heading: %.1f°", currentPose.getX(),
                currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        addVoltageCurrentTelemetry();
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

    private void addAllianceTelemetry() {
        String colorHex = Context.allianceColor == AllianceColor.RED
                ? EnhancedTelemetry.COLOR_RED
                : EnhancedTelemetry.COLOR_BLUE;
        String htmlValue = EnhancedTelemetry.htmlColor(colorHex,
                EnhancedTelemetry.htmlBold(String.valueOf(Context.allianceColor)));
        robot.telemetry.addDSRawHtml("Alliance", htmlValue);
        robot.telemetry.addDashboardData("Alliance Color", Context.allianceColor);
    }

    private void addVoltageCurrentTelemetry() {
        if (telemetryToggles.voltage) {
            double v = getVoltage();
            robot.telemetry.addDashboardData("Voltage", "%.2fV", v);
        }
        if (telemetryToggles.current) {
            double c = getTotalCurrent();
            robot.telemetry.addDashboardData("Current", "%.2fA", c);
        }
    }
}
