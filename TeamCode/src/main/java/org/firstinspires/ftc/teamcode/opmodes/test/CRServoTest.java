package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

@TeleOp(name = "CR Servo Test", group = "Test")
@Config
public class CRServoTest extends OpMode {
    public static String name = "crServo";
    public static double power = 0;
    public static boolean enabled = true;
    CRServoImplEx crServo;
    String lastName = name;

    @Override
    public void init() {
        crServo = hardwareMap.get(CRServoImplEx.class, name);
    }

    @Override
    public void loop() {
        if (enabled) {
            crServo.setPwmEnable();
            crServo.setPower(power);
        } else {
            crServo.setPwmDisable();
        }

        if (!lastName.equals(name)) {
            crServo = hardwareMap.get(CRServoImplEx.class, name);
            lastName = name;
        }
    }
}
