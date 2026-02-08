package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.hardware.SRSHub;

@TeleOp(name = "Color Sensor Test", group = "Test")
public class ColorSensorTest extends LinearOpMode {
    private SRSHub hub;

    //perhaps calibrate these with a button on init?
    double sensor1EmptyRed = 82;
    double sensor1EmptyGreen = 113;
    double sensor1EmptyBlue = 71;

    double sensor2EmptyRed = 67;
    double sensor2EmptyGreen = 121;
    double sensor2EmptyBlue = 56;

    double sensor3EmptyRed = 72;
    double sensor3EmptyGreen = 127;
    double sensor3EmptyBlue = 57;

    @Override
    public void runOpMode() {
        hub = hardwareMap.get(SRSHub.class, "srsHub");

        SRSHub.Config config = new SRSHub.Config();
        config.addI2CDevice(1, new SRSHub.APDS9151());
        config.addI2CDevice(2, new SRSHub.APDS9151());
        config.addI2CDevice(3, new SRSHub.APDS9151());
        hub.init(config);

        while (!hub.ready() && !isStopRequested()) {
            sleep(10);
        }

        waitForStart();

        while (opModeIsActive()) {

            hub.update();

            SRSHub.APDS9151 sensor1 = hub.getI2CDevice(1, SRSHub.APDS9151.class);
            SRSHub.APDS9151 sensor2 = hub.getI2CDevice(2, SRSHub.APDS9151.class);
            SRSHub.APDS9151 sensor3 = hub.getI2CDevice(3, SRSHub.APDS9151.class);


            if (gamepad1.aWasPressed()){
                calibrate(sensor1, 1);
                calibrate(sensor2, 2);
                calibrate(sensor3, 3);
            }

            DetectionResult result1 = detectGameElement(sensor1, 1);
            DetectionResult result2 = detectGameElement(sensor2, 2);
            DetectionResult result3 = detectGameElement(sensor3, 3);

            String pattern = getColorCode(result3.color) + getColorCode(result2.color)
                    + getColorCode(result1.color);

            telemetry.addData("Pattern", pattern);
            telemetry.addLine();

            telemetry.addLine("=== Sensor 1 (Bus 1) ===");
            displaySensor(sensor1, result1);
            telemetry.addLine();

            telemetry.addLine("=== Sensor 2 (Bus 2) ===");
            displaySensor(sensor2, result2);
            telemetry.addLine();

            telemetry.addLine("=== Sensor 3 (Bus 3) ===");
            displaySensor(sensor3, result3);

            telemetry.update();
        }
    }

    private void displaySensor(SRSHub.APDS9151 sensor, DetectionResult result) {
        if (sensor.disconnected) {
            telemetry.addLine("⚠ DISCONNECTED");
            return;
        }

        telemetry.addData("Red", sensor.red);
        telemetry.addData("Green", sensor.green);
        telemetry.addData("Blue", sensor.blue);
        telemetry.addData("Infrared", sensor.infrared);
        telemetry.addData("Proximity", sensor.proximity);

        int total = sensor.red + sensor.green + sensor.blue;
        if (total > 0) {
            telemetry.addData("% Red", String.format("%.1f%%", sensor.red * 100.0 / total));
            telemetry.addData("% Green", String.format("%.1f%%", sensor.green * 100.0 / total));
            telemetry.addData("% Blue", String.format("%.1f%%", sensor.blue * 100.0 / total));
            telemetry.addData("Detected", result.color);
            telemetry.addData("Green Count", result.greenCount);
            telemetry.addData("Purple Count", result.purpleCount);
        }
    }

    private static class DetectionResult {
        String color;
        int greenCount;
        int purpleCount;

        DetectionResult(String color, int greenCount, int purpleCount) {
            this.color = color;
            this.greenCount = greenCount;
            this.purpleCount = purpleCount;
        }
    }

    private DetectionResult detectGameElement(SRSHub.APDS9151 sensor, int sensorID) {

//        double initialRed = 0, initialGreen = 0, initialBlue = 0;
//
//        switch (sensorID) {
//            case 1:
//                initialRed = sensor1EmptyRed;
//                initialGreen = sensor1EmptyGreen;
//                initialBlue = sensor1EmptyBlue;
//                break;
//
//            case 2:
//                initialRed = sensor2EmptyRed;
//                initialGreen = sensor2EmptyGreen;
//                initialBlue = sensor2EmptyBlue;
//                break;
//
//            case 3:
//                initialRed = sensor3EmptyRed;
//                initialGreen = sensor3EmptyGreen;
//                initialBlue = sensor3EmptyBlue;
//                break;
//        }
//
//        double redDifference = (initialRed - sensor.red);
//        double greenDifference = (initialGreen - sensor.green);
//        double blueDifference = (initialBlue - sensor.blue);
//
//        double differenceTotal = Math.abs(redDifference) + Math.abs(greenDifference) + Math.abs(blueDifference);

//        double redRatioDiff = redDifference / total * 100;
//        double greenRatioDiff = greenDifference / total * 100;
//        double blueRatioDiff = blueDifference / total * 100;

        int total = sensor.red + sensor.green + sensor.blue;

        double redRatio = (double) sensor.red / total * 100;
        double greenRatio = (double) sensor.green / total * 100;
        double blueRatio = (double) sensor.blue / total * 100;

//        double red1Initial = (double) sensor1EmptyRed / total * 100;
//        double green1Initial = (double) sensor1EmptyGreen / total * 100;
//        double blue1Initial = (double) sensor1EmptyBlue / total * 100;
//
//        double red2Initial = (double) sensor2EmptyRed / total * 100;
//        double green2Initial = (double) sensor2EmptyGreen / total * 100;
//        double blue2Initial = (double) sensor2EmptyBlue / total * 100;
//
//        double red3Initial = (double) sensor3EmptyRed / total * 100;
//        double green3Initial = (double) sensor3EmptyGreen / total * 100;
//        double blue3Initial = (double) sensor3EmptyBlue / total * 100;
//
//        tel.addData(sensorID + " red1Initial ", "%.2f", red1Initial);
//        tel.addData(sensorID + " green1Initial ", "%.2f", green1Initial);
//        tel.addData(sensorID + " blue1Initial ", "%.2f", blue1Initial);
//        tel.addData(sensorID + " red2Initial ", "%.2f", red2Initial);
//        tel.addData(sensorID + " green2Initial ", "%.2f", green2Initial);
//        tel.addData(sensorID + " blue2Initial ", "%.2f", blue2Initial);
//        tel.addData(sensorID + " red3Initial ", "%.2f", red3Initial);
//        tel.addData(sensorID + " green3Initial ", "%.2f", green3Initial);
//        tel.addData(sensorID + " blue3Initial ", "%.2f", blue3Initial);

        int green = 0;
        int purple = 0;

        if (redRatio < 25) {
            green += 1;
        }
        if (greenRatio > 51) {
            green += 1;
        }

        if (blueRatio > 25) {
            purple += 1;
        }

        if (greenRatio < 47.5) {
            purple += 1;
        }

        telemetry.addData(sensorID + " redRatio ", "%.2f", redRatio);
        telemetry.addData(sensorID + " greenRatio ", "%.2f", greenRatio);
        telemetry.addData(sensorID + " blueRatio ", "%.2f", blueRatio);

        String color;
        if (green > purple && green > 1) {
            color = "GREEN";
        } else if (purple > green && purple > 1) {
            color = "PURPLE";
        } else {
            color = "NONE";
        }

        return new DetectionResult(color, green, purple);
    }

    private void calibrate(SRSHub.APDS9151 sensor, int sensorID) {

        switch (sensorID) {
            case 1:
                sensor1EmptyRed = sensor.red;
                sensor1EmptyGreen = sensor.green;
                sensor1EmptyBlue = sensor.blue;
                break;

            case 2:
                sensor2EmptyRed = sensor.red;
                sensor2EmptyGreen = sensor.green;
                sensor2EmptyBlue = sensor.blue;
                break;

            case 3:
                sensor3EmptyRed = sensor.red;
                sensor3EmptyGreen = sensor.green;
                sensor3EmptyBlue = sensor.blue;
                break;
        }
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
