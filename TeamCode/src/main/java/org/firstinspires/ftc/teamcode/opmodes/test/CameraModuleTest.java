package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.OpModeEx;

@TeleOp(name = "Camera Module Test", group = "Test")
@Config
@Disabled
public class CameraModuleTest extends OpModeEx {
    @Override
    public void initialize() {
        robot.follower.setPose(new Pose(24, 0, Math.toRadians(180)));
        robot.telemetry.update();
    }

    @Override
    public void onStart() {
        robot.follower.startTeleopDrive(false);
    }

    @Override
    public void primaryLoop() {
        robot.follower.setTeleOpDrive(
                -gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
    }

    @Override
    public void onEnd() {}
}
