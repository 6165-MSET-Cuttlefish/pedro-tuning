package org.firstinspires.ftc.teamcode.opmodes.test;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.control.PidflController;

@TeleOp(name = "Intake + Shooter + Magazine Test", group = "Test")
public class IntakeShooterMagazineTest extends OpMode {
    private static final double TICKS_PER_REV = 8192.0;
    private static final double STICK_DEADBAND = 0.05;
    private static final double HOOD_TRIGGER_DEADBAND = 0.05;
    private static final double HOOD_ADJUST_STEP = 0.005;
    private static final double SHOOTER_RPM_STEP = 50.0;

    public static class IntakeMagazineConfig {
        public double horizontalFrontOpen = 0.34;
        public double horizontalFrontStored = 1;
        public double horizontalBackOpen = 0.7;
        public double horizontalBackStored = 0.1;
        public double intakePower = 0;
        public double verticalPower = 0;
        public boolean isHorizontalFrontStored = false;
        public boolean isHorizontalBackStored = false;
        public boolean useGamepadIntake = true;
        public boolean useGamepadVertical = true;
    }

    public static class ShooterConfig {
        public double motorPower = 0;
        public double hoodPosition = 0.5425;
        public double targetVelocityRPM = 0;
        public double defaultVelocityRPM = 2600;
        public double highGoalRPM = 3100;
        public double closeGoalRPM = 2700;
        public double farHoodPosition = 0.735;
        public double closeHoodPosition = 0.69;
        public boolean usePid = true;
        public boolean useBangBang = true;
        public double bangBangTolerancePercent = 0.10;
    }

    public static class ShooterPid {
        public double Kp = 0.0015;
        public double Ki = 0;
        public double Kd = 0.0;
        public double FScale = 0.985;
        public double Kl = 0.0;
        public double LP1Rate = 0.7;
        public double LP2Rate = 0.3;
    }

    public static class TelemetryConfig {
        public boolean showShooter = true;
        public boolean showShooterPid = true;
        public boolean showIntake = false;
        public boolean showServos = false;
        public boolean showTurret = false;
    }

    public static class TurretConfig {
        public double targetPosition = 0.5;
        public double centerPosition = 0.5;
        public double rightPosition = 0.32;
        public double leftPosition = 0.68;
        public double tensionOffset = 0.0;
        public double positionStep = 0.02;
    }

    public static ShooterConfig shooterConfig = new ShooterConfig();
    public static IntakeMagazineConfig intakeMagazineConfig = new IntakeMagazineConfig();
    public static ShooterPid shooterPid = new ShooterPid();
    public static TurretConfig turretConfig = new TurretConfig();
    public static TelemetryConfig telemetryConfig = new TelemetryConfig();
    public static double dipTolerance = 10;
    public static double windowTimeSec = 2;

    private DcMotorEx leftMotor, rightMotor, intakeMotor, verticalMotor;
    private Servo hood, horizontalFront, horizontalBack;
    private Servo turretServoFront, turretServoBack;

    private final PidflController shooterPidController = new PidflController();
    private final ElapsedTime shooterVelocityTimer = new ElapsedTime(),
                              spinUpTimer = new ElapsedTime();

    private double shooterCurrentVelocityRPM = 0;
    private double shooterCurrentVelocityRPMRaw = 0;
    private double shooterCurrentVelocityRPMLP1, shooterCurrentVelocityRPMLP2 = 0;
    private int shooterPreviousPosition = 0;
    private boolean shooterFirstLoop = true;
    private double shooterPidOutput = 0;
    private double localMinRPM = shooterConfig.defaultVelocityRPM, spinUpTime = 0;
    private int shotsMade, shotsBounced, shotsMissed = 0, totalShots;
    private double totalLocalMinRPM, totalSpinUpTime = 0;
    private double fFinal = 0;

    @Override
    public void init() {
        initializeShooter();
        initializeIntakeMagazine();
        initializeTurret();

        shooterPidController.setController(
                shooterPid.Kp, shooterPid.Ki, shooterPid.Kd, fFinal, shooterPid.Kl);

        shooterVelocityTimer.reset();
        spinUpTimer.reset();
    }

    private void initializeShooter() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        hood = hardwareMap.get(Servo.class, "hood");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initializeIntakeMagazine() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        verticalMotor = hardwareMap.get(DcMotorEx.class, "vertical");
        horizontalFront = hardwareMap.get(Servo.class, "horizontalFront");
        horizontalBack = hardwareMap.get(Servo.class, "horizontalBack");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initializeTurret() {
        turretServoFront = hardwareMap.get(Servo.class, "turretFront");
        turretServoBack = hardwareMap.get(Servo.class, "turretBack");

        turretServoFront.setDirection(Servo.Direction.FORWARD);
        turretServoBack.setDirection(Servo.Direction.FORWARD);

        turretConfig.targetPosition = turretConfig.centerPosition;
    }

    @Override
    public void loop() {
        handleGamepadInput();
        updateShooterVelocity();

        double shooterPower = calculateShooterPower();
        applyShooterControls(shooterPower);
        applyIntakeMagazineControls();
        applyTurretControls();

        updateTelemetry(shooterPower);
    }

    private void handleGamepadInput() {
        handleDriverControls();
        handleOperatorControls();
    }

    private void handleDriverControls() {
        handleDriverIntakeControls();
        handleDriverMagazineControls();
    }

    private void handleDriverIntakeControls() {
        if (intakeMagazineConfig.useGamepadIntake) {
            double intakePower = -gamepad1.left_stick_y;
            if (Math.abs(intakePower) < STICK_DEADBAND) {
                intakePower = 0;
            }
            intakeMagazineConfig.intakePower = Range.clip(intakePower, -1.0, 1.0);
        }

        if (intakeMagazineConfig.useGamepadVertical) {
            double verticalPower = -gamepad1.right_stick_y;
            if (Math.abs(verticalPower) < STICK_DEADBAND) {
                verticalPower = 0;
            }
            intakeMagazineConfig.verticalPower = Range.clip(verticalPower, -1.0, 1.0);
        }
    }

    private void handleDriverMagazineControls() {
        if (gamepad1.leftBumperWasPressed()) {
            boolean storeBoth = !(intakeMagazineConfig.isHorizontalFrontStored
                    && intakeMagazineConfig.isHorizontalBackStored);
            intakeMagazineConfig.isHorizontalFrontStored = storeBoth;
            intakeMagazineConfig.isHorizontalBackStored = storeBoth;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            intakeMagazineConfig.isHorizontalFrontStored =
                    !intakeMagazineConfig.isHorizontalFrontStored;
        }

        if (gamepad1.dpadRightWasPressed()) {
            intakeMagazineConfig.isHorizontalBackStored =
                    !intakeMagazineConfig.isHorizontalBackStored;
        }
    }

    private void handleOperatorControls() {
        handleOperatorShooterControls();
        handleOperatorHoodControls();
        handleOperatorTurretControls();
    }

    private void handleOperatorShooterControls() {
        if (gamepad2.aWasPressed()) {
            boolean shooterActive = shooterConfig.targetVelocityRPM > 0;
            if (shooterActive) {
                stopShooter();
            } else {
                shooterConfig.targetVelocityRPM = shooterConfig.defaultVelocityRPM;
                shooterConfig.usePid = true;
            }
        }

        if (gamepad2.bWasPressed()) {
            shooterConfig.targetVelocityRPM = shooterConfig.highGoalRPM;
            shooterConfig.usePid = true;
        }

        if (gamepad2.xWasPressed()) {
            stopShooter();
        }

        if (gamepad2.leftBumperWasPressed()) {
            shooterConfig.useBangBang = !shooterConfig.useBangBang;
        }

        if (gamepad2.dpadUpWasPressed()) {
            shooterConfig.targetVelocityRPM += SHOOTER_RPM_STEP;
            shooterConfig.usePid = true;
        }

        if (gamepad2.dpadDownWasPressed()) {
            shooterConfig.targetVelocityRPM =
                    Math.max(0, shooterConfig.targetVelocityRPM - SHOOTER_RPM_STEP);
            if (shooterConfig.targetVelocityRPM == 0) {
                stopShooter();
            }
        }

        if (gamepad1.xWasPressed()) {
            shotsMade += 1;
        }
        if (gamepad1.yWasPressed()) {
            shotsBounced += 1;
        }
        if (gamepad1.bWasPressed()) {
            shotsMissed += 1;
        }

        totalShots = shotsMade + shotsBounced + shotsMissed;
    }

    private void handleOperatorHoodControls() {
        if (gamepad2.left_trigger > HOOD_TRIGGER_DEADBAND) {
            shooterConfig.hoodPosition =
                    Range.clip(shooterConfig.hoodPosition - HOOD_ADJUST_STEP, 0.0, 1.0);
        }

        if (gamepad2.right_trigger > HOOD_TRIGGER_DEADBAND) {
            shooterConfig.hoodPosition =
                    Range.clip(shooterConfig.hoodPosition + HOOD_ADJUST_STEP, 0.0, 1.0);
        }
    }

    private void handleOperatorTurretControls() {
        if (gamepad2.rightBumperWasPressed()) {
            turretConfig.targetPosition = turretConfig.centerPosition;
        }

        if (gamepad2.yWasPressed()) {
            if (Math.abs(turretConfig.targetPosition - turretConfig.centerPosition) < 0.01) {
                turretConfig.targetPosition = turretConfig.leftPosition;
            } else if (Math.abs(turretConfig.targetPosition - turretConfig.leftPosition) < 0.01) {
                turretConfig.targetPosition = turretConfig.rightPosition;
            } else {
                turretConfig.targetPosition = turretConfig.centerPosition;
            }
        }

        if (gamepad2.dpadLeftWasPressed()) {
            turretConfig.targetPosition = Range.clip(
                    turretConfig.targetPosition + turretConfig.positionStep,
                    turretConfig.rightPosition,
                    turretConfig.leftPosition);
        }

        if (gamepad2.dpadRightWasPressed()) {
            turretConfig.targetPosition = Range.clip(
                    turretConfig.targetPosition - turretConfig.positionStep,
                    turretConfig.rightPosition,
                    turretConfig.leftPosition);
        }
    }

    private double calculateShooterPower() {
        boolean usePid = shooterConfig.usePid;
        boolean useBangBang = shooterConfig.useBangBang;

        if (usePid && useBangBang) {
            return calculateShooterHybridPower();
        }

        if (usePid) {
            return calculateShooterPidPower();
        }

        if (useBangBang) {
            return calculateShooterBangBangPower();
        }

        return shooterConfig.motorPower;
    }

    private double calculateShooterHybridPower() {
        double pidPower = calculateShooterPidPower();
        double error = shooterConfig.targetVelocityRPM - shooterCurrentVelocityRPM;

        double thresholdRPM =
                shooterConfig.targetVelocityRPM * shooterConfig.bangBangTolerancePercent;

        if (error > thresholdRPM) {
            return 1.0;
        }

        if (error < -thresholdRPM) {
            return 0.0;
        }

        return pidPower;
    }

    private double calculateShooterBangBangPower() {
        double error = shooterConfig.targetVelocityRPM - shooterCurrentVelocityRPM;

        double thresholdRPM =
                shooterConfig.targetVelocityRPM * shooterConfig.bangBangTolerancePercent;

        if (error > thresholdRPM) {
            return 1.0;
        } else if (error < -thresholdRPM) {
            return 0.0;
        } else {
            return Math.max(0.0, Math.min(1.0, shooterConfig.targetVelocityRPM * fFinal));
        }
    }

    private void updateShooterVelocity() {
        int currentPosition = leftMotor.getCurrentPosition();

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

                if (shooterConfig.targetVelocityRPM - shooterCurrentVelocityRPM < dipTolerance) {
                    spinUpTimer.reset();

                } else {
                    if ((shooterCurrentVelocityRPM < localMinRPM)
                            && (spinUpTimer.seconds() < windowTimeSec)) {
                        localMinRPM = shooterCurrentVelocityRPM;
                        totalLocalMinRPM += localMinRPM;
                    }
                }

                if (spinUpTimer.seconds() > 0.08) {
                    spinUpTime = spinUpTimer.seconds();
                    totalSpinUpTime += spinUpTime;
                }
            }
        }

        shooterPreviousPosition = currentPosition;
        shooterVelocityTimer.reset();
    }

    private double calculateShooterPidPower() {
        fFinal = shooterPid.FScale * 0.0253212 * Math.sqrt(shooterConfig.targetVelocityRPM + 3626.49145) - 1.47831;
        fFinal /= shooterConfig.targetVelocityRPM;

        shooterPidController.setController(
                shooterPid.Kp, shooterPid.Ki, shooterPid.Kd, fFinal, shooterPid.Kl);
        shooterPidController.update(shooterConfig.targetVelocityRPM, shooterCurrentVelocityRPM);
        double output = shooterPidController.getPidfl();
        shooterPidOutput = output;
        return output;
    }

    private void applyShooterControls(double power) {
        double clippedPower = Math.max(-1.0, Math.min(1.0, power));
        leftMotor.setPower(clippedPower);
        rightMotor.setPower(clippedPower);
        hood.setPosition(shooterConfig.hoodPosition);
    }

    private void applyIntakeMagazineControls() {
        intakeMotor.setPower(intakeMagazineConfig.intakePower);
        verticalMotor.setPower(intakeMagazineConfig.verticalPower);
        horizontalFront.setPosition(intakeMagazineConfig.isHorizontalFrontStored
                        ? intakeMagazineConfig.horizontalFrontStored
                        : intakeMagazineConfig.horizontalFrontOpen);
        horizontalBack.setPosition(intakeMagazineConfig.isHorizontalBackStored
                        ? intakeMagazineConfig.horizontalBackStored
                        : intakeMagazineConfig.horizontalBackOpen);
    }

    private void applyTurretControls() {
        double clampedPosition = Range.clip(
                turretConfig.targetPosition,
                turretConfig.rightPosition,
                turretConfig.leftPosition);

        double frontPos = clampedPosition - turretConfig.tensionOffset;
        double backPos = clampedPosition + turretConfig.tensionOffset;

        frontPos = Range.clip(frontPos, 0.0, 1.0);
        backPos = Range.clip(backPos, 0.0, 1.0);

        turretServoFront.setPosition(frontPos);
        turretServoBack.setPosition(backPos);
    }

    private void stopShooter() {
        shooterConfig.targetVelocityRPM = 0;
        shooterConfig.motorPower = 0;
        shooterConfig.usePid = false;
        shooterConfig.useBangBang = false;
        shooterPidController.reset();
        shooterPidOutput = 0;
        shooterFirstLoop = true;
    }

    private void updateTelemetry(double shooterPower) {
        if (telemetryConfig.showShooter)
            addShooterTelemetry(shooterPower);
        if (telemetryConfig.showIntake)
            addIntakeTelemetry();
        if (telemetryConfig.showServos)
            addServoTelemetry();
        if (telemetryConfig.showTurret)
            addTurretTelemetry();

        telemetry.update();
    }

    private void addShooterTelemetry(double shooterPower) {
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Left Motor Current (A)", "%.2f", leftMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData(
                "Right Motor Current (A)", "%.2f", rightMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Position (ticks)", leftMotor.getCurrentPosition());
        telemetry.addData("Target Velocity (RPM)", "%.1f", shooterConfig.targetVelocityRPM);
        telemetry.addData("Measured Velocity (RPM)", "%.1f", shooterCurrentVelocityRPM);
        telemetry.addData("Measured Velocity LP1 (RPM)", "%.1f", shooterCurrentVelocityRPMLP1);

        telemetry.addData("Measured Velocity Raw (RPM)", "%.1f", shooterCurrentVelocityRPMRaw);
        telemetry.addData("Control Mode",
                shooterConfig.usePid && shooterConfig.useBangBang ? "PID + Bang-Bang"
                        : shooterConfig.usePid                    ? "PID"
                        : shooterConfig.useBangBang               ? "Bang-Bang"
                                                                  : "Manual");
        telemetry.addData("Final Motor Power", "%.3f", shooterPower);

        telemetry.addData("local Min RPM", "%.1f", localMinRPM);
        telemetry.addData("spin Up Time", "%.4f", spinUpTime);
        if (totalShots > 0) {
            telemetry.addData("avg RPM Dip", "%.1f", totalLocalMinRPM / totalShots);
            telemetry.addData("avg Spin Up Time", "%.4f", totalSpinUpTime / totalShots);
        }
        telemetry.addData("shots Made", shotsMade);
        telemetry.addData("shots Bounced", shotsBounced);
        telemetry.addData("shots Missed", shotsMissed);

        if (shooterConfig.usePid && telemetryConfig.showShooterPid) {
            telemetry.addData("PID Error (RPM)", "%.1f", shooterPidController.getError());
            telemetry.addData("PID Integral Sum", "%.3f", shooterPidController.getIntegralSum());
            telemetry.addData(
                    "PID Error Derivative", "%.3f", shooterPidController.getErrorDerivative());
            telemetry.addData("PID Output (Motor Power)", "%.3f", shooterPidOutput);
            telemetry.addData("F Final", "%.3f", fFinal);
        }

        if (shooterConfig.useBangBang) {
            double bbPercent = shooterConfig.bangBangTolerancePercent;
            double bbRPM = shooterConfig.targetVelocityRPM * bbPercent;

            telemetry.addData("Bang-Bang Error (RPM)", "%.1f",
                    shooterConfig.targetVelocityRPM - shooterCurrentVelocityRPM);
            telemetry.addData("Bang-Bang Tolerance (%)", "%.1f", bbPercent * 100.0);
            telemetry.addData("Bang-Bang Tolerance (RPM)", "%.1f", bbRPM);
            telemetry.addData("Bang-Bang Saturated",
                    Math.abs(shooterConfig.targetVelocityRPM - shooterCurrentVelocityRPM) > bbRPM);
        }

        if (!shooterConfig.usePid && !shooterConfig.useBangBang) {
            telemetry.addData("Shooter Motor Power", "%.3f", shooterConfig.motorPower);
        }
    }

    private void addIntakeTelemetry() {
        telemetry.addLine("=== INTAKE ===");
        telemetry.addData("Motor Current (A)", "%.2f", intakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Position (ticks)", intakeMotor.getCurrentPosition());
        telemetry.addData("Intake Motor Power", "%.3f", intakeMagazineConfig.intakePower);
        telemetry.addData("Vertical Motor Current (A)", "%.2f", verticalMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Vertical Motor Power", "%.3f", intakeMagazineConfig.verticalPower);
    }

    private void addServoTelemetry() {
        telemetry.addLine("=== SERVOS ===");
        telemetry.addData("Horizontal 1", "%s (%.3f)",
                intakeMagazineConfig.isHorizontalFrontStored ? "STORED" : "OPEN",
                horizontalFront.getPosition());
        telemetry.addData("Horizontal 2", "%s (%.3f)",
                intakeMagazineConfig.isHorizontalBackStored ? "STORED" : "OPEN",
                horizontalBack.getPosition());
        telemetry.addData("Hood Position", "%.3f", hood.getPosition());
    }

    private void addTurretTelemetry() {
        telemetry.addLine("=== TURRET ===");
        telemetry.addData("Target Position", "%.3f", turretConfig.targetPosition);
        telemetry.addData("Front Servo Position", "%.3f", turretServoFront.getPosition());
        telemetry.addData("Back Servo Position", "%.3f", turretServoBack.getPosition());
        telemetry.addData("Tension Offset", "%.3f", turretConfig.tensionOffset);
        telemetry.addData("Position Limits", "[%.2f, %.2f]",
                turretConfig.rightPosition, turretConfig.leftPosition);
    }
}
