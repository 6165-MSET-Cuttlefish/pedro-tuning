package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.teamcode.core.Robot.drivetrainTelemetry;
import static org.firstinspires.ftc.teamcode.core.Robot.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.core.Context;
import org.firstinspires.ftc.teamcode.core.ModuleEx;
import org.firstinspires.ftc.teamcode.lib.EnhancedMotor;
import org.firstinspires.ftc.teamcode.lib.State;

public class Drivetrain extends ModuleEx {
    private final EnhancedMotor fl, bl, br, fr;
    private final AnalogInput floodgate;

    public final ElapsedTime bonkTimer = new ElapsedTime();


    private final double ENCODER_TO_RPM = 60.0 / 537.7;

    public static class EnableMotors {
        public boolean enableFl = true;
        public boolean enableBl = true;
        public boolean enableBr = true;
        public boolean enableFr = true;
    }

    public static class CurrentLimiterConfig {
        public boolean enabled = true;
        public double currentThresholdMin = 25.0;
        public double currentThresholdMax = 30.0;
        public double currentOverTime = 0;
        public double minCurrentOverTime = 2;
        public double integratedCurrentLimit = 400000;
        public double decayRate = 0.9;
        public double decayLoopMs = 5;
    }

    public static EnableMotors enableMotors = new EnableMotors();
    public static CurrentLimiterConfig currentLimiterConfig = new CurrentLimiterConfig();
    private ElapsedTime currentLoopTimer;

    public enum DriveState implements State {
        MANUAL(0),
        ENDGAME(0),
        EXTERNAL(0);

        DriveState(double value) {
            setValue(value);
        }
    }

    private double flPower, blPower, brPower, frPower;
    public Drivetrain(HardwareMap hardwareMap) {
        super();
        setTelemetryEnabled(drivetrainTelemetry.TOGGLE);

        fl = new EnhancedMotor(hardwareMap, "fl")
                .withCachingTolerance(0.05);
//                .withVoltageCompensation(13.5);
        bl = new EnhancedMotor(hardwareMap, "bl")
                .withCachingTolerance(0.05);
//                .withVoltageCompensation(13.5);
        fr = new EnhancedMotor(hardwareMap, "fr")
                .withCachingTolerance(0.05);
//                .withVoltageCompensation(13.5);
        br = new EnhancedMotor(hardwareMap, "br")
                .withCachingTolerance(0.05);
//                .withVoltageCompensation(13.5);


        floodgate = hardwareMap.get(AnalogInput.class, "floodgate");
    }

    @Override
    public void init(){
        super.init();
        currentLoopTimer = new ElapsedTime();
    }
    @Override
    protected void initStates() {
        setStates(DriveState.MANUAL);
    }

    @Override
    protected void read() {
        if (!enableMotors.enableFl)
            flPower = 0;
        if (!enableMotors.enableBl)
            blPower = 0;
        if (!enableMotors.enableBr)
            brPower = 0;
        if (!enableMotors.enableFr)
            frPower = 0;
    }

    @Override
    protected void write() {
        if (!get(DriveState.class).equals(DriveState.EXTERNAL)) {
            fl.setPower(flPower);
            bl.setPower(blPower);
            br.setPower(brPower);
            fr.setPower(frPower);
        }
    }

    public void setTargets(double fl, double bl, double br, double fr) {
        double maxPower = Math.max(
                Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));

        if (maxPower > 1.0) {
            fl /= maxPower;
            bl /= maxPower;
            fr /= maxPower;
            br /= maxPower;
        }

        double scale = getCurrentLimiterMultiplier();
        flPower = fl * scale;
        blPower = bl * scale;
        brPower = br * scale;
        frPower = fr * scale;
        currentLoopTimer.reset();
    }

    public void setRawTargets(double fl, double bl, double br, double fr) {
        flPower = fl;
        blPower = bl;
        brPower = br;
        frPower = fr;
    }

    public void stop() {
        setTargets(0, 0, 0, 0);
    }

    public boolean isBonk() {
        if (robot.follower.isBusy() && !robot.follower.getCurrentPath().isAtParametricEnd() && robot.follower.getVelocity().getMagnitude() < 2) {
            if (bonkTimer.milliseconds() > 2000) {
                return true;
            }
        } else {
            bonkTimer.reset();
        }
        return false;
    }

    public double getFloodgateCurrent() {
        double voltage = floodgate.getVoltage();
        return (voltage / 3.3) * 80.0;
    }

    public double getCurrentLimiterMultiplier() {
//        if (!currentLimiterConfig.enabled) {
//            return 1.0;
//        }

        double current = robot.opMode.getTotalCurrent();
        currentLimiterConfig.currentOverTime += Math.pow(current, 2) * currentLoopTimer.milliseconds();
        currentLimiterConfig.currentOverTime *= Math.pow(currentLimiterConfig.decayRate, (currentLoopTimer.milliseconds() / currentLimiterConfig.decayLoopMs));
        robot.telemetry.addDashboardData("currentOverTime", currentLimiterConfig.currentOverTime);
        robot.telemetry.addDashboardData("final current scale", 1.0 - currentLimiterConfig.currentOverTime / currentLimiterConfig.integratedCurrentLimit);


        if (current <= currentLimiterConfig.currentThresholdMin) { //25
            return 1.0;
        }
//        if (current >= currentLimiterConfig.currentThresholdMax) { //30
//            return 0.0;
//        }
        double range = currentLimiterConfig.currentThresholdMax - currentLimiterConfig.currentThresholdMin;
        double excess = current - currentLimiterConfig.currentThresholdMin;
//        return 1.0 - (excess / range);

        //theoretical integral current scaling. used 400000 because theoretically (I^2 * t) shouldn't exceed that
        return 1.0 - Range.clip(currentLimiterConfig.currentOverTime / currentLimiterConfig.integratedCurrentLimit, 0, 1);


    }

    public void setMecanumTargets(double y, double x, double rx, boolean fieldCentric) {
        if (fieldCentric) {
            double heading = -robot.follower.getPose().getHeading();
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);

            if (Context.allianceColor == AllianceColor.BLUE) {
                y = -y;
            }

            double rotatedX = x * cos - y * sin;
            double rotatedY = x * sin + y * cos;

            y = rotatedY;
            x = rotatedX;
        }

        double frontLeft = y + x + rx;
        double backLeft = y - x + rx;
        double frontRight = y - x - rx;
        double backRight = y + x - rx;

        setTargets(frontLeft, backLeft, backRight, frontRight);
    }

    public EnhancedMotor getFl() {
        return fl;
    }

    public EnhancedMotor getBl() {
        return bl;
    }

    public EnhancedMotor getBr() {
        return br;
    }

    public EnhancedMotor getFr() {
        return fr;
    }

    @Override
    protected void onTelemetry() {
        if (drivetrainTelemetry.TOGGLE) {
            logDashboard("Drive Mode", get(DriveState.class));
            logDashboard("Motor Powers", "FL:%.2f BL:%.2f FR:%.2f BR:%.2f", flPower, blPower, frPower, brPower);

            logDashboard("FL Power", fl.getPower());
            logDashboard("BL Power", bl.getPower());
            logDashboard("BR Power", br.getPower());
            logDashboard("FR Power", fr.getPower());
            logDashboard("FL Position (ticks)", fl.getCurrentPosition());
            logDashboard("FR Position (ticks)", fr.getCurrentPosition());
            logDashboard("FL Velocity (RPM)", fl.getVelocity() * ENCODER_TO_RPM);
            logDashboard("BL Velocity (RPM)", bl.getVelocity() * ENCODER_TO_RPM);
            logDashboard("BR Velocity (RPM)", br.getVelocity() * ENCODER_TO_RPM);
            logDashboard("FR Velocity (RPM)", fr.getVelocity() * ENCODER_TO_RPM);

            logDashboard("Floodgate Current (A)", "%.2f", getFloodgateCurrent());
            logDashboard("Current Limiter Multiplier", "%.2f", getCurrentLimiterMultiplier());

            if (drivetrainTelemetry.current) {
                logDashboard("FL Current (A)", fl.getCurrent(CurrentUnit.AMPS));
                logDashboard("BL Current (A)", bl.getCurrent(CurrentUnit.AMPS));
                logDashboard("BR Current (A)", br.getCurrent(CurrentUnit.AMPS));
                logDashboard("FR Current (A)", fr.getCurrent(CurrentUnit.AMPS));
            }
        }
    }
}
