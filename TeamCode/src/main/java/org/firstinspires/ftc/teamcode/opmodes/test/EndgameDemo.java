package org.firstinspires.ftc.teamcode.opmodes.test;


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
import org.firstinspires.ftc.teamcode.lib.Action;
import org.firstinspires.ftc.teamcode.lib.Actions;

@TeleOp(name = "Endgame Demo", group = "Test")
public class EndgameDemo extends OpMode {
    public static class HardwareConfig {
        public String flName = "fl";
        public String blName = "bl";
        public String leftPtoName = "leftPto";
        public String leftInitialName = "leftInitial";
        public String leftInitialEncoderName = "leftInitialEncoder";
    }

    public static class MotorControl {
        public boolean runToPosition = false;
        public boolean useRawPower = false;
        public int targetTicks = 0;
        public double motorPower = 0;
        public boolean usePidfl = true;
    }

    public static class PidflConfig {
        public double leftMultiplier = 1;

        public double leftP = 0.01;
        public double leftI = 0.0;
        public double leftD = 0.0002;
        public double leftF = 0.0;
        public double leftL = 0.0;

        public int leftTargetPosition = 0;

        public boolean enableLinearDeceleration = false;
        public int decelerationDistance = 1000;
        public double minimumDecelerationPower = 0.5;
    }

    public static class ServoControl {
        public double leftPtoPosition = 0.45; // down = 1
        public double leftInitialLiftPower = 0;
        public boolean turnOnLeftInitialLift = true;
    }

    public static class InitialLiftPidflConfig {
        public boolean usePidfl = false;

        public double leftMultiplier = 1;

        public double leftP = 0.005;
        public double leftI = 0.0;
        public double leftD = 0.0002;
        public double leftF = 0.0;
        public double leftL = 0.0;

        public double leftTargetPosition = 0; // degrees

        public boolean enableLinearDeceleration = false;
        public double decelerationDistance = 10.0; // degrees
        public double minimumDecelerationPower = 0.1;
    }

    public static class GamepadConfig {
        public int motorLiftUpTarget = -5000;

        public double ptoEngagedPosition = 1.0;

        public double initialLiftUpTarget = 700.0;

        public long ptoEngageDelayMs = 500;
        public long initialLiftTimeoutMs = 3000;

        public boolean runEndgameSequence = false;
    }

    public static HardwareConfig hardwareConfig = new HardwareConfig();
    public static MotorControl motorControl = new MotorControl();
    public static ServoControl servoControl = new ServoControl();
    public static PidflConfig pidflConfig = new PidflConfig();
    public static InitialLiftPidflConfig initialLiftPidflConfig = new InitialLiftPidflConfig();
    public static GamepadConfig gamepadConfig = new GamepadConfig();

    DcMotorEx fl;
    DcMotorEx bl;
    ServoImplEx leftPto;
    CRServoImplEx leftInitial;
    AbsoluteAnalogEncoder leftInitialEncoder;

    PidflController leftPidfl;
    PidflController leftInitialPidfl;

    private boolean motorsPoweredDown = false;
    private boolean initialLiftPoweredDown = false;
    private boolean lastRunEndgameSequence = false;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotorEx.class, hardwareConfig.flName);
        bl = hardwareMap.get(DcMotorEx.class, hardwareConfig.blName);
        leftPto = hardwareMap.get(ServoImplEx.class, hardwareConfig.leftPtoName);
        leftInitial = hardwareMap.get(CRServoImplEx.class, hardwareConfig.leftInitialName);
        leftInitialEncoder = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, hardwareConfig.leftInitialEncoderName), 0, 1.0, false);

        initializeDriveMotor(fl);
        initializeDriveMotor(bl);

        leftPidfl = new PidflController();
        leftInitialPidfl = new PidflController();

        // Reset all control state to defaults to prevent auto-movement on restart
        motorControl.usePidfl = false;
        motorControl.runToPosition = false;
        motorControl.useRawPower = false;
        motorControl.motorPower = 0;
        motorControl.targetTicks = 0;
        pidflConfig.leftTargetPosition = 0;

        servoControl.leftPtoPosition = 0.45;
        servoControl.leftInitialLiftPower = 0;
        servoControl.turnOnLeftInitialLift = true;

        initialLiftPidflConfig.usePidfl = false;
        initialLiftPidflConfig.leftTargetPosition = 0;
    }

    @Override
    public void init_loop() {
        leftInitialEncoder.zero();
    }

    @Override
    public void loop() {
        handleGamepadControls();

        // Reset power-down flags when control modes are active
        if (motorControl.usePidfl || motorControl.runToPosition || motorControl.useRawPower) {
            motorsPoweredDown = false;
        }
        if (initialLiftPidflConfig.usePidfl || servoControl.leftInitialLiftPower != 0) {
            initialLiftPoweredDown = false;
        }

        // Motor Control
        if (motorControl.usePidfl) {
            // Update PID coefficients
            leftPidfl.setController(pidflConfig.leftP, pidflConfig.leftI, pidflConfig.leftD,
                    pidflConfig.leftF, pidflConfig.leftL);

            // Set target
            leftPidfl.setTarget(pidflConfig.leftTargetPosition);

            // Update position
            leftPidfl.updatePosition(fl.getCurrentPosition());

            // Calculate power
            double leftPower = leftPidfl.getPidfl();

            // Apply linear deceleration when near target
            if (pidflConfig.enableLinearDeceleration) {
                leftPower = applyLinearDeceleration(leftPower, leftPidfl.getError(),
                        pidflConfig.decelerationDistance, pidflConfig.minimumDecelerationPower);
            }

            // Apply power
            fl.setPower(leftPower * pidflConfig.leftMultiplier);
            bl.setPower(leftPower * pidflConfig.leftMultiplier);

        } else if (motorControl.runToPosition) {
            fl.setTargetPosition(motorControl.targetTicks);
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fl.setPower(motorControl.motorPower);
            bl.setPower(motorControl.motorPower);

        } else if (motorControl.useRawPower) {
            fl.setPower(motorControl.motorPower);
            bl.setPower(motorControl.motorPower);

        } else {
            // Set power to 0 once when all control modes are disabled
            if (!motorsPoweredDown) {
                fl.setPower(0);
                bl.setPower(0);
                motorsPoweredDown = true;
            }
        }

        // Servo Control
        leftPto.setPosition(servoControl.leftPtoPosition);

        // Initial Lift Control
        if (initialLiftPidflConfig.usePidfl) {
            // Update PID coefficients
            leftInitialPidfl.setController(initialLiftPidflConfig.leftP, initialLiftPidflConfig.leftI,
                    initialLiftPidflConfig.leftD, initialLiftPidflConfig.leftF, initialLiftPidflConfig.leftL);

            // Set target
            leftInitialPidfl.setTarget(initialLiftPidflConfig.leftTargetPosition);

            // Update position
            leftInitialPidfl.updatePosition(leftInitialEncoder.getPosition());

            // Calculate power
            double leftInitialPower = leftInitialPidfl.getPidfl();

            // Apply linear deceleration when near target
            if (initialLiftPidflConfig.enableLinearDeceleration) {
                leftInitialPower = applyLinearDeceleration(leftInitialPower, leftInitialPidfl.getError(),
                        initialLiftPidflConfig.decelerationDistance, initialLiftPidflConfig.minimumDecelerationPower);
            }

            // Apply power
            setInitialLiftPower(leftInitial, servoControl.turnOnLeftInitialLift,
                    leftInitialPower * initialLiftPidflConfig.leftMultiplier);
        } else if (servoControl.leftInitialLiftPower != 0) {
            setInitialLiftPower(leftInitial, servoControl.turnOnLeftInitialLift,
                    servoControl.leftInitialLiftPower);
        } else if (!initialLiftPoweredDown) {
            // Power down initial lift once when control mode is disabled
            leftInitial.setPower(0);
            initialLiftPoweredDown = true;
        }

        // Telemetry
        String controlMode = motorControl.usePidfl
                ? "PIDFL"
                : (motorControl.runToPosition ? "RUN_TO_POSITION" : "MANUAL");
        telemetry.addData("Control Mode", controlMode);

        telemetry.addData("FL Current (A)", fl.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BL Current (A)", bl.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("FL Position (ticks)", fl.getCurrentPosition());

        if (motorControl.usePidfl) {
            telemetry.addData("Left Target", pidflConfig.leftTargetPosition);
            telemetry.addData("Left Error", leftPidfl.getError());
            telemetry.addData("Left Power", leftPidfl.getPidfl());
        } else if (motorControl.runToPosition) {
            telemetry.addData("Target Ticks (Reference: FL)", motorControl.targetTicks);
            telemetry.addData("Motor Power", motorControl.motorPower);
            telemetry.addData("Reference Motor (FL) Busy", fl.isBusy());
            telemetry.addData("Target Reached", !fl.isBusy());
        } else {
            telemetry.addData("Motor Power", motorControl.motorPower);
        }

        telemetry.addData("Left Servo Position", servoControl.leftPtoPosition);

        telemetry.addData("Initial Lift Control Mode",
                initialLiftPidflConfig.usePidfl ? "PIDFL" : "MANUAL");

        telemetry.addData("Left Initial Encoder (deg)", leftInitialEncoder.getPosition());
        telemetry.addData("Left Initial Target", initialLiftPidflConfig.leftTargetPosition);
        telemetry.addData("Left Initial Error", leftInitialPidfl.getError());
        telemetry.addData("Left Initial Power", leftInitialPidfl.getPidfl());
        telemetry.addData("Left Initial Lift Power", servoControl.leftInitialLiftPower);

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

    private void handleGamepadControls() {
        // Gamepad trigger
        if (gamepad1.xWasPressed()) {
            endgameSequence().run();
        }

        // FTC Dashboard trigger (edge detection)
        if (gamepadConfig.runEndgameSequence && !lastRunEndgameSequence) {
            endgameSequence().run();
        }
        lastRunEndgameSequence = gamepadConfig.runEndgameSequence;

        telemetry.addLine();
        telemetry.addData("Press X or toggle runEndgameSequence in FTC Dashboard", "Run endgame sequence");
    }

    /**
     * Complete endgame sequence:
     * 1. Raise initial lift to target
     * 2. Wait for initial lift to complete
     * 3. Engage PTO (servo down)
     * 4. Delay for PTO engagement
     * 5. Disable initial lift servo
     * 6. Raise full lift using PIDFL
     */
    private Action endgameSequence() {
        return Actions.builder()
                .run(() -> {
                    // Start initial lift
                    initialLiftPidflConfig.usePidfl = true;
                    initialLiftPidflConfig.leftTargetPosition = gamepadConfig.initialLiftUpTarget;
                    servoControl.turnOnLeftInitialLift = true;
                })
                .waitUntil(this::isInitialLiftAtTarget, gamepadConfig.initialLiftTimeoutMs)
                .run(() -> {
                    // Engage PTO
                    servoControl.leftPtoPosition = gamepadConfig.ptoEngagedPosition;
                })
                .delay(gamepadConfig.ptoEngageDelayMs)
                .run(() -> {
                    // Reset motor encoders
                    fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                })
                .run(() -> {
                    // Disable initial lift and start full lift
                    servoControl.leftInitialLiftPower = 0;
                    servoControl.turnOnLeftInitialLift = false;
                    initialLiftPidflConfig.usePidfl = false;

                    // Enable PIDFL for full lift
                    motorControl.usePidfl = true;
                    motorControl.runToPosition = false;
                    motorControl.useRawPower = false;
                    pidflConfig.leftTargetPosition = gamepadConfig.motorLiftUpTarget;
                })
                .build();
    }

    private boolean isInitialLiftAtTarget() {
        double error = Math.abs(leftInitialEncoder.getPosition() - initialLiftPidflConfig.leftTargetPosition);
        return error < 20.0;
    }
}
