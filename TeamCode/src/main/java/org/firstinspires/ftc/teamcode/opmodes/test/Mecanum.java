package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(group = "Test")
public class Mecanum extends OpMode {
    public static boolean currentTelemetry = false;

    ElapsedTime loopTime = new ElapsedTime();
    DcMotorEx fl, fr, br, bl, intake;
    public static double lastFlPower, lastFrPower, lastBrPower, lastBlPower;

    public static double epsilon = 0.01;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        br = hardwareMap.get(DcMotorEx.class, "br");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        intake = hardwareMap.get(DcMotorEx.class, "bl");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double flPower = y + x + rx;
        double blPower = y - x + rx;
        double frPower = y - x - rx;
        double brPower = y + x - rx;

        if (Math.abs(flPower - lastFlPower) > epsilon) {
            fl.setPower(flPower);
            lastFlPower = flPower;
        }
        if (Math.abs(brPower - lastBrPower) > epsilon) {
            br.setPower(brPower);
            lastBrPower = brPower;
        }
        if (Math.abs(frPower - lastFrPower) > epsilon) {
            fr.setPower(frPower);
            lastFrPower = frPower;
        }
        if (Math.abs(blPower - lastBlPower) > epsilon) {
            bl.setPower(blPower);
            lastBlPower = blPower;
        }

        if (gamepad1.right_trigger_pressed) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

        telemetry.addData("fl", fl.getPower());
        telemetry.addData("bl", fl.getPower());
        telemetry.addData("fr", fl.getPower());
        telemetry.addData("br", fl.getPower());

        if (currentTelemetry) {
            telemetry.addData("fl Current", fl.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("bl Current", bl.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("fr Current", fr.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("br Current", br.getCurrent(CurrentUnit.AMPS));
        }

        telemetry.addData("Loop time", loopTime.milliseconds());
        loopTime.reset();
    }
}
