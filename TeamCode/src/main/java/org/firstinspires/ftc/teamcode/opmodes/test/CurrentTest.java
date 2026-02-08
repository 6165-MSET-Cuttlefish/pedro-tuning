package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "Current Test", group = "Test")
public class CurrentTest extends LinearOpMode {
    public static boolean controlHubStall = false;
    public static boolean expansionHubStall = false;
    public static boolean stopStall = false;
    public static double threshold = 5;
    double lastCurrent = 0;

    private static final String TAG = "CurrentTesting";

    ElapsedTime timer;
    public VoltageSensor voltageSensor;
    public double voltage;
    MultipleTelemetry tel;

    @Override
    public void runOpMode() {
        DcMotorEx motorControl0 = hardwareMap.get(DcMotorEx.class, "motorControl0");
        DcMotorEx motorControl1 = hardwareMap.get(DcMotorEx.class, "motorControl1");
        DcMotorEx motorControl2 = hardwareMap.get(DcMotorEx.class, "motorControl2");
        DcMotorEx motorControl3 = hardwareMap.get(DcMotorEx.class, "motorControl3");

        DcMotorEx motorExpansion0 = hardwareMap.get(DcMotorEx.class, "motorExpansion0");
        DcMotorEx motorExpansion1 = hardwareMap.get(DcMotorEx.class, "motorExpansion1");
        DcMotorEx motorExpansion2 = hardwareMap.get(DcMotorEx.class, "motorExpansion2");
        DcMotorEx motorExpansion3 = hardwareMap.get(DcMotorEx.class, "motorExpansion3");

        timer = new ElapsedTime();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            if (gamepad1.a || controlHubStall) {
                controlHubStall = false;

                motorControl0.setPower(1);
                motorControl1.setPower(1);
                motorControl2.setPower(1);
                motorControl3.setPower(1);
            }

            if (gamepad1.b || expansionHubStall) {
                expansionHubStall = false;

                motorExpansion0.setPower(1);
                motorExpansion1.setPower(1);
                motorExpansion2.setPower(1);
                motorExpansion3.setPower(1);
            }

            if (gamepad1.x || stopStall) {
                stopStall = false;

                motorControl0.setPower(0);
                motorControl1.setPower(0);
                motorControl2.setPower(0);
                motorControl3.setPower(0);
                motorExpansion0.setPower(0);
                motorExpansion1.setPower(0);
                motorExpansion2.setPower(0);
                motorExpansion3.setPower(0);
            }

            double current = 0.0;
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                current += module.getCurrent(CurrentUnit.AMPS);
            }

            if (current > threshold && lastCurrent <= threshold) {
                timer.reset();
            }

            voltage = voltageSensor.getVoltage();

            tel.addData("current (A)", "%.2f", current);
            tel.addData("lastCurrent (A)", "%.2f", lastCurrent);
            tel.addData("voltage (V)", "%.2f", voltage);
            tel.addData("timer (ms since cross)", "%.0f", timer.milliseconds());
            tel.update();

            RobotLog.ii(TAG, "current=%.2f A, last=%.2f A, voltage=%.2f V, timer=%.0f ms", current,
                    lastCurrent, voltage, timer.milliseconds());

            lastCurrent = current;
        }
    }
}
