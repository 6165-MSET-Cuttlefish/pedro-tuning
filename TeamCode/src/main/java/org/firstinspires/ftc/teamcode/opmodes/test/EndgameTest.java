package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.control.PidflController;
import org.firstinspires.ftc.teamcode.architecture.hardware.AbsoluteAnalogEncoder;

@Config
@TeleOp(name = "Endgame Test", group = "Test")
public class EndgameTest extends OpMode {
    public static class HardwareConfig {
        public String flName = "fl";
        public String frName = "fr";
        public String brName = "br";
        public String blName = "bl";
        public String leftPtoName = "leftPto";
        public String rightPtoName = "rightPto";
        public String leftInitialName = "leftInitial";
        public String rightInitialName = "rightInitial";
        public String leftInitialEncoderName = "leftInitialEncoder";
        public String rightInitialEncoderName = "rightInitialEncoder";
    }

    public static class MotorControl {
        public boolean runToPosition = false;
        public boolean useRawPower = false;
        public int targetTicks = 0;
        public double motorPower = 0;
        public boolean usePidfl = true;
    }

    public static class PidflConfig {
        public boolean syncTargetPositions = false;
        public int syncedTargetPosition = 0;

        public double leftMultiplier = 1;
        public double rightMultiplier = 1;

        public double leftP = 0.01;
        public double leftI = 0.0;
        public double leftD = 0.0002;
        public double leftF = 0.0;
        public double leftL = 0.0;

        public double rightP = 0.01;
        public double rightI = 0.0;
        public double rightD = 0.0002;
        public double rightF = 0.0;
        public double rightL = 0.0;

        public int leftTargetPosition = 0;
        public int rightTargetPosition = 0;

        public boolean enableLinearDeceleration = false;
        public int decelerationDistance = 1000;
        public double minimumDecelerationPower = 0.5;
    }

    public static class ServoControl {
        public double leftPtoPosition = 0.005; // down = 1
        public double rightPtoPosition = 0.6175; // down = 0
        public boolean syncInitialLiftPower = false;
        public double syncedInitialLiftPower = 0;
        public double leftInitialLiftPower = 0;
        public double rightInitialLiftPower = 0;
        public boolean turnOnLeftInitialLift = true;
        public boolean turnOnRightInitialLift = true;
    }

    public static class InitialLiftPidflConfig {
        public boolean usePidfl = false;
        public boolean syncTargetPositions = false;
        public double syncedTargetPosition = 0;

        public double leftMultiplier = 1;
        public double rightMultiplier = 1;

        public double leftP = 0.005;
        public double leftI = 0.0;
        public double leftD = 0.0002;
        public double leftF = 0.0;
        public double leftL = 0.0;

        public double rightP = 0.005;
        public double rightI = 0.0;
        public double rightD = 0.0002;
        public double rightF = 0.0;
        public double rightL = 0.0;

        public double leftTargetPosition = 0; // degrees
        public double rightTargetPosition = 0; // degrees

        public boolean enableLinearDeceleration = false;
        public double decelerationDistance = 10.0; // degrees
        public double minimumDecelerationPower = 0.1;
    }

    public static HardwareConfig hardwareConfig = new HardwareConfig();
    public static MotorControl motorControl = new MotorControl();
    public static ServoControl servoControl = new ServoControl();
    public static PidflConfig pidflConfig = new PidflConfig();
    public static InitialLiftPidflConfig initialLiftPidflConfig = new InitialLiftPidflConfig();

    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx br;
    DcMotorEx bl;
    ServoImplEx leftPto;
    ServoImplEx rightPto;
    CRServoImplEx leftInitial;
    CRServoImplEx rightInitial;
    AbsoluteAnalogEncoder leftInitialEncoder;
    AbsoluteAnalogEncoder rightInitialEncoder;

    PidflController leftPidfl;
    PidflController rightPidfl;
    PidflController leftInitialPidfl;
    PidflController rightInitialPidfl;

    private boolean motorsPoweredDown = false;
    private boolean initialLiftPoweredDown = false;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotorEx.class, hardwareConfig.flName);
        fr = hardwareMap.get(DcMotorEx.class, hardwareConfig.frName);
        br = hardwareMap.get(DcMotorEx.class, hardwareConfig.brName);
        bl = hardwareMap.get(DcMotorEx.class, hardwareConfig.blName);
        leftPto = hardwareMap.get(ServoImplEx.class, hardwareConfig.leftPtoName);
        rightPto = hardwareMap.get(ServoImplEx.class, hardwareConfig.rightPtoName);
        leftInitial = hardwareMap.get(CRServoImplEx.class, hardwareConfig.leftInitialName);
        rightInitial = hardwareMap.get(CRServoImplEx.class, hardwareConfig.rightInitialName);
        leftInitialEncoder = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, hardwareConfig.leftInitialEncoderName), 0, 1.0, false);
        rightInitialEncoder = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, hardwareConfig.rightInitialEncoderName), 0, 1.0, false);

        initializeDriveMotor(fl);
        initializeDriveMotor(fr);
        initializeDriveMotor(br);
        initializeDriveMotor(bl);

        leftPidfl = new PidflController();
        rightPidfl = new PidflController();
        leftInitialPidfl = new PidflController();
        rightInitialPidfl = new PidflController();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() {
        leftInitialEncoder.zero();
        rightInitialEncoder.zero();
    }

    @Override
    public void loop() {
        // Reset power-down flags when control modes are active
        if (motorControl.usePidfl || motorControl.runToPosition || motorControl.useRawPower) {
            motorsPoweredDown = false;
        }
        if (initialLiftPidflConfig.usePidfl || servoControl.syncInitialLiftPower) {
            initialLiftPoweredDown = false;
        }

        // Motor Control
        if (motorControl.usePidfl) {
            // Update PID coefficients
            leftPidfl.setController(pidflConfig.leftP, pidflConfig.leftI, pidflConfig.leftD,
                    pidflConfig.leftF, pidflConfig.leftL);
            rightPidfl.setController(pidflConfig.rightP, pidflConfig.rightI, pidflConfig.rightD,
                    pidflConfig.rightF, pidflConfig.rightL);

            // Set targets
            if (pidflConfig.syncTargetPositions) {
                leftPidfl.setTarget(pidflConfig.syncedTargetPosition);
                rightPidfl.setTarget(pidflConfig.syncedTargetPosition);
            } else {
                leftPidfl.setTarget(pidflConfig.leftTargetPosition);
                rightPidfl.setTarget(pidflConfig.rightTargetPosition);
            }

            // Update positions
            leftPidfl.updatePosition(fl.getCurrentPosition());
            rightPidfl.updatePosition(fr.getCurrentPosition());

            // Calculate power
            double leftPower = leftPidfl.getPidfl();
            double rightPower = rightPidfl.getPidfl();

            // Apply linear deceleration when near target
            if (pidflConfig.enableLinearDeceleration) {
                leftPower = applyLinearDeceleration(leftPower, leftPidfl.getError(),
                        pidflConfig.decelerationDistance, pidflConfig.minimumDecelerationPower);
                rightPower = applyLinearDeceleration(rightPower, rightPidfl.getError(),
                        pidflConfig.decelerationDistance, pidflConfig.minimumDecelerationPower);
            }

            // Apply power
            fl.setPower(leftPower * pidflConfig.leftMultiplier);
            bl.setPower(leftPower * pidflConfig.leftMultiplier);
            fr.setPower(rightPower * pidflConfig.rightMultiplier);
            br.setPower(rightPower * pidflConfig.rightMultiplier);

        } else if (motorControl.runToPosition) {
            fl.setTargetPosition(motorControl.targetTicks);
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fl.setPower(motorControl.motorPower);
            fr.setPower(motorControl.motorPower);
            br.setPower(motorControl.motorPower);
            bl.setPower(motorControl.motorPower);

        } else if (motorControl.useRawPower) {
            fl.setPower(motorControl.motorPower);
            fr.setPower(motorControl.motorPower);
            br.setPower(motorControl.motorPower);
            bl.setPower(motorControl.motorPower);

        } else {
            // Set power to 0 once when all control modes are disabled
            if (!motorsPoweredDown) {
                fl.setPower(0);
                fr.setPower(0);
                br.setPower(0);
                bl.setPower(0);
                motorsPoweredDown = true;
            }
        }

        // Servo Control
        leftPto.setPosition(servoControl.leftPtoPosition);
        rightPto.setPosition(servoControl.rightPtoPosition);

        // Initial Lift Control
        if (initialLiftPidflConfig.usePidfl) {
            // Update PID coefficients
            leftInitialPidfl.setController(initialLiftPidflConfig.leftP, initialLiftPidflConfig.leftI,
                    initialLiftPidflConfig.leftD, initialLiftPidflConfig.leftF, initialLiftPidflConfig.leftL);
            rightInitialPidfl.setController(initialLiftPidflConfig.rightP, initialLiftPidflConfig.rightI,
                    initialLiftPidflConfig.rightD, initialLiftPidflConfig.rightF, initialLiftPidflConfig.rightL);

            // Set targets
            if (initialLiftPidflConfig.syncTargetPositions) {
                leftInitialPidfl.setTarget(initialLiftPidflConfig.syncedTargetPosition);
                rightInitialPidfl.setTarget(initialLiftPidflConfig.syncedTargetPosition);
            } else {
                leftInitialPidfl.setTarget(initialLiftPidflConfig.leftTargetPosition);
                rightInitialPidfl.setTarget(initialLiftPidflConfig.rightTargetPosition);
            }

            // Update positions
            leftInitialPidfl.updatePosition(leftInitialEncoder.getPosition());
            rightInitialPidfl.updatePosition(-rightInitialEncoder.getPosition());

            // Calculate power
            double leftInitialPower = leftInitialPidfl.getPidfl();
            double rightInitialPower = rightInitialPidfl.getPidfl();

            // Apply linear deceleration when near target
            if (initialLiftPidflConfig.enableLinearDeceleration) {
                leftInitialPower = applyLinearDeceleration(leftInitialPower, leftInitialPidfl.getError(),
                        initialLiftPidflConfig.decelerationDistance, initialLiftPidflConfig.minimumDecelerationPower);
                rightInitialPower = applyLinearDeceleration(rightInitialPower, rightInitialPidfl.getError(),
                        initialLiftPidflConfig.decelerationDistance, initialLiftPidflConfig.minimumDecelerationPower);
            }

            // Apply power
            setInitialLiftPower(leftInitial, servoControl.turnOnLeftInitialLift,
                    leftInitialPower * initialLiftPidflConfig.leftMultiplier);
            setInitialLiftPower(rightInitial, servoControl.turnOnRightInitialLift,
                    -rightInitialPower * initialLiftPidflConfig.rightMultiplier);
        } else if (servoControl.syncInitialLiftPower) {
            setInitialLiftPower(leftInitial, servoControl.turnOnLeftInitialLift,
                    servoControl.syncedInitialLiftPower);
            setInitialLiftPower(rightInitial, servoControl.turnOnRightInitialLift,
                    -servoControl.syncedInitialLiftPower);
        } else if (servoControl.leftInitialLiftPower != 0 || servoControl.rightInitialLiftPower != 0) {
            setInitialLiftPower(leftInitial, servoControl.turnOnLeftInitialLift,
                    servoControl.leftInitialLiftPower);
            setInitialLiftPower(rightInitial, servoControl.turnOnRightInitialLift,
                    -servoControl.rightInitialLiftPower);
        } else if (!initialLiftPoweredDown) {
            // Power down initial lift once when both control modes are disabled
            leftInitial.setPower(0);
            rightInitial.setPower(0);
            initialLiftPoweredDown = true;
        }

        // Telemetry
        String controlMode = motorControl.usePidfl
                ? "PIDFL"
                : (motorControl.runToPosition ? "RUN_TO_POSITION" : "MANUAL");
        telemetry.addData("Control Mode", controlMode);

        telemetry.addData("FL Current (A)", fl.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("FR Current (A)", fr.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BR Current (A)", br.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BL Current (A)", bl.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("FL Position (ticks)", fl.getCurrentPosition());
        telemetry.addData("FR Position (ticks)", fr.getCurrentPosition());

        if (motorControl.usePidfl) {
            telemetry.addData(
                    "Sync Mode", pidflConfig.syncTargetPositions ? "ENABLED" : "DISABLED");
            if (pidflConfig.syncTargetPositions) {
                telemetry.addData("Synced Target", pidflConfig.syncedTargetPosition);
                int leftPos = fl.getCurrentPosition();
                int rightPos = fr.getCurrentPosition();
                telemetry.addData("Position Error", leftPos - rightPos);
            } else {
                telemetry.addData("Left Target", pidflConfig.leftTargetPosition);
                telemetry.addData("Right Target", pidflConfig.rightTargetPosition);
            }
            telemetry.addData("Left Error", leftPidfl.getError());
            telemetry.addData("Right Error", rightPidfl.getError());
            telemetry.addData("Left Power", leftPidfl.getPidfl());
            telemetry.addData("Right Power", rightPidfl.getPidfl());
        } else if (motorControl.runToPosition) {
            telemetry.addData("Target Ticks (Reference: FL)", motorControl.targetTicks);
            telemetry.addData("Motor Power (All Motors)", motorControl.motorPower);
            telemetry.addData("Reference Motor (FL) Busy", fl.isBusy());
            telemetry.addData("Target Reached", !fl.isBusy());
        } else {
            telemetry.addData("Motor Power", motorControl.motorPower);
        }

        telemetry.addData("Left Servo Position", servoControl.leftPtoPosition);
        telemetry.addData("Right Servo Position", servoControl.rightPtoPosition);

        telemetry.addData("Initial Lift Control Mode",
                initialLiftPidflConfig.usePidfl ? "PIDFL" : (servoControl.syncInitialLiftPower ? "SYNC_POWER" : "MANUAL"));

        telemetry.addData("Left Initial Encoder (deg)", leftInitialEncoder.getPosition());
        telemetry.addData("Right Initial Encoder (deg)", -rightInitialEncoder.getPosition());
        telemetry.addData("Initial Lift Sync Mode",
                initialLiftPidflConfig.syncTargetPositions ? "ENABLED" : "DISABLED");
        if (initialLiftPidflConfig.syncTargetPositions) {
            telemetry.addData("Synced Initial Target", initialLiftPidflConfig.syncedTargetPosition);
        } else {
            telemetry.addData("Left Initial Target", initialLiftPidflConfig.leftTargetPosition);
            telemetry.addData("Right Initial Target", initialLiftPidflConfig.rightTargetPosition);
        }
        telemetry.addData("Left Initial Error", leftInitialPidfl.getError());
        telemetry.addData("Right Initial Error", rightInitialPidfl.getError());
        telemetry.addData("Left Initial Power", leftInitialPidfl.getPidfl());
        telemetry.addData("Right Initial Power", rightInitialPidfl.getPidfl());

        if (servoControl.syncInitialLiftPower) {
            telemetry.addData("Synced Initial Lift Power", servoControl.syncedInitialLiftPower);
        } else {
            telemetry.addData("Left Initial Lift Power", servoControl.leftInitialLiftPower);
            telemetry.addData("Right Initial Lift Power", -servoControl.rightInitialLiftPower);
        }

        telemetry.update();
    }

    private void initializeDriveMotor(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double applyLinearDeceleration(double power, double error, double decelerationDistance,
                                           double minimumPower) {
        double absError = Math.abs(error);
        if (absError <= decelerationDistance && absError > 0) {
            double decelerationFactor = absError / decelerationDistance;
            return minimumPower + (power - minimumPower) * decelerationFactor;
        }
        return power;
    }

    private void setInitialLiftPower(CRServoImplEx servo, boolean enabled, double power) {
        if (enabled) {
            servo.setPwmEnable();
            servo.setPower(power);
        } else {
            servo.setPwmDisable();
        }
    }
}
