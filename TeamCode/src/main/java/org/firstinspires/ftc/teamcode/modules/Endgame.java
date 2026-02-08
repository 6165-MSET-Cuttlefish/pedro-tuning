package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.architecture.control.PidflController;
import org.firstinspires.ftc.teamcode.architecture.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.core.ModuleEx;
import org.firstinspires.ftc.teamcode.lib.State;

import static org.firstinspires.ftc.teamcode.core.Robot.endgameTelemetry;
import static org.firstinspires.ftc.teamcode.core.Robot.robot;

@Config
public class Endgame extends ModuleEx {
    private ServoImplEx leftPto;
    private ServoImplEx rightPto;
    private CRServoImplEx leftInitial;
    private CRServoImplEx rightInitial;

    public AbsoluteAnalogEncoder leftInitialEncoder;
    public AbsoluteAnalogEncoder rightInitialEncoder;


    private PidflController leftPidfl;
    private PidflController rightPidfl;
    private PidflController leftInitialPidfl;
    private PidflController rightInitialPidfl;
    public int fullLiftTimeMs = 4000;

    public static class FullLiftConfig {
        public double leftMultiplier = 1;
        public double rightMultiplier = 1;
        public double leftPower;
        public double rightPower;
    }

    public static class InitialLiftConfig {
        public double leftMultiplier = 1;
        public double rightMultiplier = 0.8;

        public boolean enableLinearDeceleration = false;
        public double decelerationDistance = 10.0; // degrees
        public double minimumDecelerationPower = 0.1;
    }

    public static class LeftPIDFLConfig {
        public double p = 0.001;
        public double i = 0.0;
        public double d = 0.0002;
        public double f = 0.0;
        public double l = 0.0;
    }

    public static class RightPIDFLConfig {
        public double p = 0.001;
        public double i = 0.0;
        public double d = 0.0002;
        public double f = 0.0;
        public double l = 0.0;
    }

    public static class LeftInitialPIDFLConfig {
        public double p = 0.0025;
        public double i = 0.0;
        public double d = 0.0002;
        public double f = 0.0;
        public double l = 0.0;
    }

    public static class RightInitialPIDFLConfig {
        public double p = 0.0025;
        public double i = 0.0;
        public double d = 0.0002;
        public double f = 0.0;
        public double l = 0.0;
    }

    public static LeftPIDFLConfig leftPidflConfig = new LeftPIDFLConfig();
    public static RightPIDFLConfig rightPidflConfig = new RightPIDFLConfig();
    public static InitialLiftConfig initialLiftConfig = new InitialLiftConfig();
    public static LeftInitialPIDFLConfig leftInitialPidflConfig = new LeftInitialPIDFLConfig();
    public static RightInitialPIDFLConfig rightInitialPidflConfig = new RightInitialPIDFLConfig();
    public static FullLiftConfig fullLiftConfig = new FullLiftConfig();

    public enum FullLiftState implements State {
        INIT(0),
        FULL_LIFT(-11000);

        FullLiftState(double value) {
            setValue(value);
        }
    }

    public enum LeftPtoState implements State {
        UP(0.005),
        DOWN(1),
        MANUAL(-1);

        LeftPtoState(double value) {
            setValue(value);
        }
    }

    public enum RightPtoState implements State {
        UP(0.6175),
        DOWN(0),
        MANUAL(-1);

        RightPtoState(double value) {
            setValue(value);
        }
    }

    public enum InitialState implements State {
        LIFT(700),
        DISABLED(LIFT.getValue());

        InitialState(double value) {
            setValue(value);
        }
    }


    private double leftPtoPosition = 0.0;
    private double rightPtoPosition = 0.0;


    public double fullLiftTargetPosition = 0;

    public Endgame(HardwareMap hardwareMap) {
        super();
        setTelemetryEnabled(endgameTelemetry.TOGGLE);

        leftPto = hardwareMap.get(ServoImplEx.class, "leftPto");
        rightPto = hardwareMap.get(ServoImplEx.class, "rightPto");
        leftInitial = hardwareMap.get(CRServoImplEx.class, "leftInitial");
        rightInitial = hardwareMap.get(CRServoImplEx.class, "rightInitial");
        leftInitialEncoder = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "leftInitialEncoder"), 0, 1.0, false);
        rightInitialEncoder = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "rightInitialEncoder"), 0, 1.0, false);

        leftPidfl = new PidflController();
        rightPidfl = new PidflController();
        leftInitialPidfl = new PidflController();
        rightInitialPidfl = new PidflController();
    }

    @Override
    protected void initStates() {
        setStates(FullLiftState.INIT, LeftPtoState.UP, RightPtoState.UP, InitialState.DISABLED);
    }

    @Override
    protected void read() {
        leftPtoPosition = get(LeftPtoState.class).getValue();
        rightPtoPosition = get(RightPtoState.class).getValue();


        leftInitialPidfl.setController(
                leftInitialPidflConfig.p, leftInitialPidflConfig.i, leftInitialPidflConfig.d, leftInitialPidflConfig.f, leftInitialPidflConfig.l);
        rightInitialPidfl.setController(
                rightInitialPidflConfig.p, rightInitialPidflConfig.i, rightInitialPidflConfig.d, rightInitialPidflConfig.f, rightInitialPidflConfig.l);
        leftPidfl.setController(
                leftPidflConfig.p, leftPidflConfig.i, leftPidflConfig.d, leftPidflConfig.f, leftPidflConfig.l);
        rightPidfl.setController(
                rightPidflConfig.p, rightPidflConfig.i, rightPidflConfig.d, rightPidflConfig.f, rightPidflConfig.l);

        leftInitialPidfl.update(get(InitialState.class).getValue(), leftInitialEncoder.getPosition());
        rightInitialPidfl.update(get(InitialState.class).getValue(), -rightInitialEncoder.getPosition());

        fullLiftTargetPosition = get(FullLiftState.class).getValue();
        leftPidfl.update(fullLiftTargetPosition, -robot.drivetrain.getFl().getCurrentPosition());
        rightPidfl.update(fullLiftTargetPosition, robot.drivetrain.getFr().getCurrentPosition());

        double t = (double) robot.endgameDecelTimer.milliseconds() / fullLiftTimeMs;
        t = Math.min(t, 1.0);
        double decelScale = sCurve(1.0 - t);

        fullLiftConfig.leftPower = leftPidfl.getPidfl() * fullLiftConfig.leftMultiplier;// * decelScale;
        fullLiftConfig.rightPower = rightPidfl.getPidfl() * fullLiftConfig.rightMultiplier;// * decelScale;
    }

    public double sCurve(double t) {
        // t in range [0,1]
        return 0.5 * (1 - Math.cos(Math.PI * t));
    }

    @Override
    protected void write() {
        if (get(InitialState.class).equals(InitialState.DISABLED)) {
            leftInitial.setPower(0);
            rightInitial.setPower(0);
            if (leftInitial.isPwmEnabled()) {
                leftInitial.setPwmDisable();
            }
            if (rightInitial.isPwmEnabled()) {
                rightInitial.setPwmDisable();
            }
        } else {
            if (!leftInitial.isPwmEnabled()) {
                leftInitial.setPwmEnable();
            }
            if (!rightInitial.isPwmEnabled()) {
                rightInitial.setPwmEnable();
            }
            leftInitial.setPower(leftInitialPidfl.getPidfl() * initialLiftConfig.leftMultiplier);
            rightInitial.setPower(-rightInitialPidfl.getPidfl() * initialLiftConfig.rightMultiplier);
        }

        if (get(FullLiftState.class).equals(FullLiftState.FULL_LIFT)) {
            robot.drivetrain.setRawTargets(-fullLiftConfig.leftPower, -fullLiftConfig.leftPower,
                    fullLiftConfig.rightPower, fullLiftConfig.rightPower);
        }

        leftPto.setPosition(get(LeftPtoState.class).getValue());
        rightPto.setPosition(get(RightPtoState.class).getValue());
    }

    public boolean initialLiftComplete() {
        // need to retune
        return Math.abs(leftInitialEncoder.getPosition() - get(InitialState.class).getValue()) < 70 &&
                Math.abs(-rightInitialEncoder.getPosition() - get(InitialState.class).getValue()) < 70;
    }

    @Override
    protected void onTelemetry() {
        if (endgameTelemetry.TOGGLE) {

            // PID telemetry
            logDashboard("Left PID Power", leftPidfl.getPidfl());
            logDashboard("Right PID Power", rightPidfl.getError());
            logDashboard("Left PID Error", leftPidfl.getError());
            logDashboard("Right PID Error", rightPidfl.getError());

            if (endgameTelemetry.pto) {
                logDashboard("Left PTO State", get(LeftPtoState.class));
                logDashboard("Right PTO State", get(RightPtoState.class));
                logDashboard("Left PTO Position", leftPtoPosition);
                logDashboard("Right PTO Position", rightPtoPosition);
            }

            if (endgameTelemetry.initial) {
                logDashboard("Left Initial Encoder", leftInitialEncoder.getPosition());
                logDashboard("Right Initial Encoder", -rightInitialEncoder.getPosition());
                logDashboard("Left Initial PID Error", leftInitialPidfl.getError());
                logDashboard("Right Initial PID Error", rightInitialPidfl.getError());
                logDashboard("Left Initial PID Power", leftInitialPidfl.getPidfl());
                logDashboard("Right Initial PID Power", rightInitialPidfl.getPidfl());
                logDashboard("InitialState", get(InitialState.class).getValue());
            }
        }
    }
}
