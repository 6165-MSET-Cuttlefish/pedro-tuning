package org.firstinspires.ftc.teamcode.opmodes.test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Brushland Test", group = "Test")
public class BrushlandTest extends LinearOpMode {
    private AnalogInput analogPin0;
    private AnalogInput analogPin1;
    private AnalogInput analogPin2;

    @Override
    public void runOpMode() {
        analogPin0 = hardwareMap.analogInput.get("analog0");
        analogPin1 = hardwareMap.analogInput.get("analog1");
        analogPin2 = hardwareMap.analogInput.get("analog2");

        waitForStart();

        while (opModeIsActive()) {
            double voltage0 = analogPin0.getVoltage();
            double voltage1 = analogPin1.getVoltage();
            double voltage2 = analogPin2.getVoltage();

            double hue0 = voltage0 / 3.3 * 360;
            double hue1 = voltage1 / 3.3 * 360;
            double hue2 = voltage2 / 3.3 * 360;

            DetectionResult result0 = detectColor(hue0);
            DetectionResult result1 = detectColor(hue1);
            DetectionResult result2 = detectColor(hue2);

            String pattern = getColorCode(result2.color) + getColorCode(result1.color)
                    + getColorCode(result0.color);

            telemetry.addData("Pattern", pattern);
            telemetry.addLine();

            telemetry.addLine("=== Sensor 0 (Analog 0) ===");
            displaySensor(voltage0, hue0, result0);
            telemetry.addLine();

            telemetry.addLine("=== Sensor 1 (Analog 1) ===");
            displaySensor(voltage1, hue1, result1);
            telemetry.addLine();

            telemetry.addLine("=== Sensor 2 (Analog 2) ===");
            displaySensor(voltage2, hue2, result2);

            telemetry.update();
        }
    }

    private void displaySensor(double voltage, double hue, DetectionResult result) {
        telemetry.addData("Voltage (V)", String.format("%.2f", voltage));
        telemetry.addData("Hue (0-360°)", String.format("%.1f", hue));
        telemetry.addData("Detected Color", result.color);
    }

    private static class DetectionResult {
        String color;

        DetectionResult(String color) {
            this.color = color;
        }
    }

    private DetectionResult detectColor(double hue) {
        int green = 0;
        int purple = 0;

        if (hue >= 100 && hue <= 140) {
            green += 1;
        }

        if (hue >= 80 && hue <= 160) {
            green += 1;
        }

        if (hue >= 270 && hue <= 300) {
            purple += 1;
        }

        if (hue >= 260 && hue <= 310) {
            purple += 1;
        }

        String color;
        if (green > purple && green > 1) {
            color = "GREEN";
        } else if (purple > green && purple > 1) {
            color = "PURPLE";
        } else {
            color = "NONE";
        }

        return new DetectionResult(color);
    }

    private String getColorCode(String color) {
        if (color.equals("GREEN")) {
            return "G";
        } else if (color.equals("PURPLE")) {
            return "P";
        } else {
            return "N";
        }
    }
}
