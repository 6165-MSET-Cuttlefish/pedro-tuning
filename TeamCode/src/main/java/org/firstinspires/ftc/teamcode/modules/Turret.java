package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.architecture.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.core.Context;
import org.firstinspires.ftc.teamcode.core.ModuleEx;
import org.firstinspires.ftc.teamcode.lib.EnhancedServo;
import org.firstinspires.ftc.teamcode.lib.State;

import static org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization.toField;
import static org.firstinspires.ftc.teamcode.core.Robot.robot;
import static org.firstinspires.ftc.teamcode.core.Robot.turretTelemetry;

@Config
public class Turret extends ModuleEx {
    public static double turretOffset = 0;
    public static double POSITION_TOLERANCE_DEGREES = 1.0;
    public static double turretX = -5.35;
    public static double turretY = -1.175;
    public static double TENSION_OFFSET = 0.0;
    public static boolean robotVelocityCorrection = true;

    private final EnhancedServo turretServoFront;
    private final EnhancedServo turretServoBack;

    public double targetAngle = 0.0;
    public double targetServoPosition = 0.5;
    public double lastTargetServoPosition = 0.5;

    public static class TurretConfig {
        public double gearRatio = 1.0 / 3.0;
        public double flightTime = 1;
    }

    public static TurretConfig turretConfig = new TurretConfig();

    public enum TurretState implements State {
        CENTER(0.5),
        RIGHT(0.33),
        LEFT(0.67),
        AUTOAIM(-1),
        HOLD(-1),
        OFF(-1);

        TurretState(double value) {
            setValue(value);
        }
    }

    public Turret(HardwareMap hardwareMap) {
        super();
        setTelemetryEnabled(turretTelemetry.TOGGLE);

        turretServoFront = new EnhancedServo(hardwareMap, "turretFront")
                .withCachingTolerance(0.001);
        turretServoBack = new EnhancedServo(hardwareMap, "turretBack")
                .withCachingTolerance(0.001);

        turretServoFront.setDirection(Servo.Direction.FORWARD);
        turretServoBack.setDirection(Servo.Direction.FORWARD);

        turretOffset = 0;
    }

    @Override
    protected void initStates() {
        setStates(TurretState.AUTOAIM);
    }

    @Override
    protected void read() {
        updateTargetPosition();
    }

    @Override
    protected void write() {
        if (get(TurretState.class) == TurretState.OFF) {
            return;
        }

        double frontPos = targetServoPosition - TENSION_OFFSET;
        double backPos = targetServoPosition + TENSION_OFFSET;

        frontPos = Math.max(0.0, Math.min(1.0, frontPos));
        backPos = Math.max(0.0, Math.min(1.0, backPos));

        turretServoFront.setPosition(frontPos);
        turretServoBack.setPosition(backPos);
    }

    @Override
    protected void onTelemetry() {
        if (turretTelemetry.TOGGLE) {
            if (turretTelemetry.position) {
                logDashboard("Turret State", get(TurretState.class));
                log("Target Angle (deg)", String.format("%.1f", targetAngle));
                logDashboard("Target Servo Position", String.format("%.3f", targetServoPosition));
                logDashboard("Turret Offset", turretOffset);
            }

            if (turretTelemetry.servos) {
                logDashboard("Front Servo Position", turretServoFront.getPosition());
                logDashboard("Back Servo Position", turretServoBack.getPosition());
            }
        }
    }

    private void updateTargetPosition() {
        if (get(TurretState.class).equals(TurretState.AUTOAIM)) {
            Pose robotPose = robot.follower.getPose();
            double robotHeading = Math.toDegrees(robotPose.getHeading());

            double robotRad = Math.toRadians(robotHeading);
            double turretFieldX =
                    robotPose.getX() + turretX * Math.cos(robotRad) - turretY * Math.sin(robotRad);
            double turretFieldY =
                    robotPose.getY() + turretX * Math.sin(robotRad) + turretY * Math.cos(robotRad);

            double turretDashX = toField(new Pose(turretFieldX, turretFieldY)).getX();
            double turretDashY = toField(new Pose(turretFieldX, turretFieldY)).getY();

            robot.packet.fieldOverlay().setStroke("#FFFFFF")
                    .fillCircle(turretDashX, turretDashY, 2);

            double targetX = robot.targetPose.getX();
            double targetY = robot.targetPose.getY();

            if (robotVelocityCorrection) {
                Vector robotVelocity = robot.follower.getVelocity();
                double robotVx = robotVelocity.getXComponent();
                double robotVy = robotVelocity.getYComponent();

                targetX = targetX - robotVx * turretConfig.flightTime;
                targetY = targetY - robotVy * turretConfig.flightTime;
            }

            double targetDashX = toField(new Pose(targetX, targetY)).getX();
            double targetDashY = toField(new Pose(targetX, targetY)).getY();

            robot.packet.fieldOverlay().setStroke(Context.allianceColor.equals(AllianceColor.BLUE) ? "blue" : "red")
                    .fillCircle(targetDashX, targetDashY, 2);

            double absoluteAngle =
                    Math.toDegrees(Math.atan2(targetY - turretFieldY, targetX - turretFieldX));

            targetAngle = absoluteAngle - robotHeading;
            targetAngle = normalizeAngle(targetAngle);
            targetAngle += turretOffset;

            double angle = normalizeAngle(targetAngle);
            if (angle > 180) {
                targetServoPosition = 0.5
                        + (TurretState.LEFT.getValue() - TurretState.RIGHT.getValue())
                                / 180 * (angle - 360);
            } else {
                targetServoPosition = 0.5
                        + (TurretState.LEFT.getValue() - TurretState.RIGHT.getValue())
                                / 180 * (angle);
            }
        } else if (get(TurretState.class).equals(TurretState.HOLD)) {
            targetServoPosition = lastTargetServoPosition;
        }
        else {
            targetServoPosition = get(TurretState.class).getValue();
        }

        if (targetServoPosition > TurretState.LEFT.getValue()) {
            targetServoPosition = TurretState.LEFT.getValue();
        } else if (targetServoPosition < TurretState.RIGHT.getValue()) {
            targetServoPosition = TurretState.RIGHT.getValue();
        }
        lastTargetServoPosition = targetServoPosition;
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    private double calculateShortestError(double target, double current) {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    public double getTargetAngle() {
        return targetAngle;
    }
}
