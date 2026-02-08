package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.teamcode.core.Robot.robot;
import static org.firstinspires.ftc.teamcode.core.Robot.shooterTelemetry;
import static org.firstinspires.ftc.teamcode.modules.Turret.turretConfig;
import static org.firstinspires.ftc.teamcode.modules.Turret.turretX;
import static org.firstinspires.ftc.teamcode.modules.Turret.turretY;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.control.PidflController;
import org.firstinspires.ftc.teamcode.core.ModuleEx;
import org.firstinspires.ftc.teamcode.lib.EnhancedMotor;
import org.firstinspires.ftc.teamcode.lib.EnhancedServo;
import org.firstinspires.ftc.teamcode.lib.State;

@Config
public class Shooter extends ModuleEx {
    private static final double TICKS_PER_REV = 8192.0;
    private static final double VELOCITY_TOLERANCE_RPM = 60.0;
    public static double maxRobotVelocity = 3;
    public static boolean robotVelocityCorrection = true;

    private final EnhancedMotor left;
    private final EnhancedMotor right;
    private final EnhancedServo hood;

    public double targetVelocityRPM = 0;
    public double hoodPosition;

    public double velocityOffset = 0;
    public double hoodOffset = 0;

    public static class ShooterPID {
        public double Kp = 0.0015;
        public double Ki = 0;
        public double Kd = 0.00025;
        public double Kf = 0.000265;
        public double FScale = 1;
        public double Kl = 0.0;
        public double LP1Rate = 0.7;
        public double LP2Rate = 0.3;
    }

    public static double bangBangTolerancePercent = 0.10;
    public static ShooterPID shooterPid = new ShooterPID();

    private final PidflController shooterPidController = new PidflController();
    private final ElapsedTime shooterVelocityTimer = new ElapsedTime();
    private final ElapsedTime spinUpTimer = new ElapsedTime();

    private double shooterCurrentVelocityRPM = 0;
    private double shooterCurrentVelocityRPMRaw = 0;
    private double shooterCurrentVelocityRPMLP1, shooterCurrentVelocityRPMLP2 = 0;
    private int shooterPreviousPosition = 0;
    private boolean shooterFirstLoop = true;
    private double shooterPidOutput = 0;

    public enum FlywheelState implements State {
        IDLE(1800),
        FAR(3150.0), // middle of closer tape: 3250 rpm, 0.82 hood; 3100 rpm, 0.78 hood // middle of further tape: 3350 rpm, 0.82 hood; 3100 rpm, 0.78 hood
        FAR_AUTO(3150.0),
        CLOSE(2950.0),
        CLOSE_AUTO(2400.0),
        LOW(1500),
        OFF(0),
        MANUAL(0.2),
        COAST_TO_TARGET(0),
        PID(0);

        FlywheelState(double value) {
            setValue(value);
        }
    }

    // Ball sequence tracking for multi-ball shots
    public int ballInSequence = 2; // 1, 2, or 3
    public double distanceToGoal = 0; // Current distance to goal in inches

    public enum HoodState implements State {
        RESET(0.585),
        BOTTOM(0),
        SUPER_CLOSE(0.31),
        KINDA_CLOSE(0.28),
        FAR(0.15),
        CLOSE(0.19),
        CLOSE_AUTO(.060),
        TOP(0.3),
        MANUAL(-1),
        PID(0);

        HoodState(double value) {
            setValue(value);
        }
    }

    public Shooter(HardwareMap hardwareMap) {
        super();
        setTelemetryEnabled(shooterTelemetry.TOGGLE);

        left = new EnhancedMotor(hardwareMap, "leftFlywheel");
        right = new EnhancedMotor(hardwareMap, "rightFlywheel");
        hood = new EnhancedServo(hardwareMap, "hood");

        left.setVoltageCompensationEnabled(true);
        right.setVoltageCompensationEnabled(true);
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterPidController.setController(
                shooterPid.Kp, shooterPid.Ki, shooterPid.Kd, shooterPid.Kf, shooterPid.Kl);
        shooterVelocityTimer.reset();
        spinUpTimer.reset();
    }

    @Override
    protected void initStates() {
        setStates(FlywheelState.OFF, HoodState.BOTTOM);
    }

    @Override
    protected void read() {
        updateShooterVelocity();

        Pose robotPose = robot.follower.getPose();
        double robotRad = robotPose.getHeading();
        double turretFieldX =
                robotPose.getX() + turretX * Math.cos(robotRad) - turretY * Math.sin(robotRad);
        double turretFieldY =
                robotPose.getY() + turretX * Math.sin(robotRad) + turretY * Math.cos(robotRad);

        double targetX = robot.cornerPose.getX();
        double targetY = robot.cornerPose.getY();
        if (robotVelocityCorrection) {
            Vector robotVelocity = robot.follower.getVelocity();
            double robotVx = robotVelocity.getXComponent();
            double robotVy = robotVelocity.getYComponent();

            targetX = targetX - robotVx * turretConfig.flightTime;
            targetY = targetY - robotVy * turretConfig.flightTime;
            distanceToGoal = Math.hypot(targetX - turretFieldX, targetY - turretFieldY);
        } else {
            distanceToGoal = Math.hypot(targetX - turretFieldX, targetY - turretFieldY);
        }

        if (get(FlywheelState.class).equals(FlywheelState.PID)) {
            targetVelocityRPM = ShooterInterpolation.getTargetRPM(distanceToGoal);
        } else {
            targetVelocityRPM = get(FlywheelState.class).getValue();
        }

        if (get(HoodState.class).equals(HoodState.PID)) {
            hoodPosition = ShooterInterpolation.getHoodPosition(distanceToGoal, shooterCurrentVelocityRPM) + HoodState.RESET.getValue();
        } else {
            if (!get(HoodState.class).equals(HoodState.RESET)) {
                hoodPosition = get(HoodState.class).getValue() + (HoodState.RESET).getValue();
            }
        }

        targetVelocityRPM += velocityOffset;
        hoodPosition += hoodOffset;

        calculateShooterPower();
    }

    @Override
    protected void write() {
        left.setPower(shooterPidOutput);
        right.setPower(shooterPidOutput);

        if (hoodPosition >= HoodState.RESET.getValue()
                || hoodPosition <= HoodState.TOP.getValue() + HoodState.RESET.getValue()) {
            hood.setPosition(hoodPosition);
        }
    }

    @Override
    protected void onTelemetry() {
        if (shooterTelemetry.TOGGLE) {
            if (shooterTelemetry.flywheel) {
                logDashboard("Flywheel State", get(FlywheelState.class));
                log("Target Velocity (RPM)", String.format("%.1f", targetVelocityRPM));
                log("Measured Velocity (RPM)", String.format("%.1f", shooterCurrentVelocityRPM));
                logDashboard("Measured Velocity LP1 (RPM)",
                        String.format("%.1f", shooterCurrentVelocityRPMLP1));
                logDashboard("Measured Velocity Raw (RPM)",
                        String.format("%.1f", shooterCurrentVelocityRPMRaw));

                logDashboard("PID Error (RPM)", String.format("%.1f", shooterPidController.getError()));
                logDashboard("PID Output", String.format("%.3f", shooterPidOutput));

                double bbRPM = targetVelocityRPM * bangBangTolerancePercent;
                logDashboard("Bang-Bang Error (RPM)",
                        String.format("%.1f", targetVelocityRPM - shooterCurrentVelocityRPM));
                logDashboard("Bang-Bang Tolerance (RPM)", String.format("%.1f", bbRPM));

                logDashboard("Distance to Goal", distanceToGoal);

                // Add this inside the shooterTelemetry.flywheel block in onTelemetry()
                if (shooterTelemetry.flywheel) {
                    // ... existing flywheel telemetry ...

                    // Interpolation telemetry
                    logDashboard("Interp Target Distance", String.format("%.1f", ShooterInterpolation.lastTargetDistance));
                    logDashboard("Interp Base RPM", String.format("%.1f", ShooterInterpolation.lastBaseRPM));
                    logDashboard("Interp Compensated RPM", String.format("%.1f", ShooterInterpolation.lastCompensatedRPM));
                    logDashboard("Interp Selected Hood", String.format("%.3f", ShooterInterpolation.lastSelectedHood));
                    logDashboard("Interp Distance Index", String.format("%d", ShooterInterpolation.lastClosestDistanceIndex));
                    logDashboard("Interp Points Count", String.format("%d", ShooterInterpolation.lastPointsCount));
                }

            }

            if (shooterTelemetry.hood) {
                logDashboard("Hood State", get(HoodState.class));
                log("Hood Position", String.format("%.3f", hoodPosition));
            }

            if (shooterTelemetry.current) {
                logDashboard("Left Flywheel Current (A)",
                        String.format("%.2f", left.getCurrent(CurrentUnit.AMPS)));
                logDashboard("Right Flywheel Current (A)",
                        String.format("%.2f", right.getCurrent(CurrentUnit.AMPS)));
            }
        }
    }

    private void updateShooterVelocity() {
        int currentPosition = left.getCurrentPosition();

        if (shooterFirstLoop) {
            shooterCurrentVelocityRPM = 0;
            shooterFirstLoop = false;
        } else {
            double deltaTime = shooterVelocityTimer.seconds();
            if (deltaTime > 0) {
                double deltaPosition = currentPosition - shooterPreviousPosition;
                double rawRPM = (deltaPosition / deltaTime) * (60.0 / TICKS_PER_REV);

                shooterCurrentVelocityRPMRaw = rawRPM;

                shooterCurrentVelocityRPMLP1 +=
                        (rawRPM - shooterCurrentVelocityRPMLP1) * shooterPid.LP1Rate;
                shooterCurrentVelocityRPMLP2 +=
                        (shooterCurrentVelocityRPMLP1 - shooterCurrentVelocityRPMLP2)
                                * shooterPid.LP2Rate;

                shooterCurrentVelocityRPM = shooterCurrentVelocityRPMLP2;
            }
        }

        shooterPreviousPosition = currentPosition;
        shooterVelocityTimer.reset();
    }

    private void calculateShooterPower() {
        double pidPower = calculateShooterPIDPower();
        double error = targetVelocityRPM - shooterCurrentVelocityRPM;
        double thresholdRPM = targetVelocityRPM * bangBangTolerancePercent;

        if (error > thresholdRPM) {
            shooterPidOutput = 1;
        } else if (error < -thresholdRPM) {
            shooterPidOutput = 0;
        } else {
            shooterPidOutput = pidPower;
        }

        if (get(FlywheelState.class).equals(FlywheelState.OFF)) {
            shooterPidOutput = 0;
        }
    }

    private double calculateShooterPIDPower() {
        if (targetVelocityRPM != 0) {
            // https://www.desmos.com/calculator/ykvsfthqvf
            double f = shooterPid.FScale * 0.0253212 * Math.sqrt(targetVelocityRPM + 3626.49145) - 1.47831;
            shooterPid.Kf = f / targetVelocityRPM;
        }
        shooterPidController.setController(
                shooterPid.Kp, shooterPid.Ki, shooterPid.Kd, shooterPid.Kf, shooterPid.Kl);
        shooterPidController.update(targetVelocityRPM, shooterCurrentVelocityRPM);
        return shooterPidController.getPidfl();
    }

    public double getCurrentVelocityRPM() {
        return shooterCurrentVelocityRPM;
    }

    public double getCurrentVelocityRPMRaw() {
        return shooterCurrentVelocityRPMRaw;
    }

    public double getTargetVelocityRPM() {
        return targetVelocityRPM;
    }

    public boolean isAtTargetVelocity() {
        if (targetVelocityRPM > 0) {
            double velocityError = Math.abs(targetVelocityRPM - shooterCurrentVelocityRPM);
            return velocityError <= VELOCITY_TOLERANCE_RPM;
        }
        return true;
    }
}