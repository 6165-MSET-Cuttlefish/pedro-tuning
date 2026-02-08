package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.teamcode.core.Robot.magazineTelemetry;
import static org.firstinspires.ftc.teamcode.core.Robot.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.hardware.SRSHub;
import org.firstinspires.ftc.teamcode.core.ModuleEx;
import org.firstinspires.ftc.teamcode.lib.EnhancedMotor;
import org.firstinspires.ftc.teamcode.lib.EnhancedServo;
import org.firstinspires.ftc.teamcode.lib.State;

public class Magazine extends ModuleEx {
    private final EnhancedServo horizontalFront;
    private final EnhancedServo horizontalBack;
    private final EnhancedMotor intake;
    private final EnhancedMotor vertical;
    private final EnhancedServo headlightLeft;
    private final EnhancedServo headlightRight;
    private SRSHub.APDS9151 sensor1, sensor2, sensor3;
    public double horizontalFrontPosition, horizontalBackPosition;
    public double intakePower, verticalPower;
    public static double headlightPosition;
    public String colorPattern;
    private MagazineState currentState;
    private boolean isValid;
    private double strobePosition = 0.28;
    private boolean strobeIncreasing = true;
    private long lastStrobeUpdate = 0;
    private static final double STROBE_MIN = 0.28;
    private static final double STROBE_MAX = 0.71;
    private static final double STROBE_SPEED = 0.05;
    private static final long STROBE_INTERVAL_MS = 50;

    public static int transferTimeShootAll = 100;
    public static int horizontalTime = 900;
    public static int transferTime = 250;
    public static int shootTime = 750;


    public enum HorizontalFrontState implements State {
        OPEN(0.34),
        STORED(1),
        MANUAL(-1);

        HorizontalFrontState(double value) { setValue(value); }
    }

    public enum HorizontalBackState implements State {
        OPEN(0.325),
        SLIGHT_CLOSED(0.4),
        STORED(0.65),
        MANUAL(-1);

        HorizontalBackState(double value) { setValue(value); }
    }

    public enum IntakeState implements State {
        FORWARD(1),
        HALF(0.5),
        REVERSE(-1),
        OFF(0),
        IDLE(0.5),
        MANUAL(0);

        IntakeState(double value) { setValue(value); }
    }

    public enum VerticalState implements State {
        ON(1),
        HALF_DOWN(-0.35),
        OFF(0);

        VerticalState(double value) { setValue(value); }
    }


    public enum HeadlightState implements State {
        OFF(0),
        RED(0.28),
        GREEN(0.46),
        CYAN(0.6),
        CYAN_STROBE(0.6),
        YELLOW(0.39),
        WHITE(1),
        WHITE_STROBE(1),
        STROBE(-1);

        HeadlightState(double value) { setValue(value); }
    }

    public enum SortState implements State {
        UNSORTED(0),
        PREPARED(1),
        SORTED(2);

        SortState(double value) { setValue(value); }
    }

    public Magazine(HardwareMap hardwareMap) {
        super();
        setTelemetryEnabled(magazineTelemetry.TOGGLE);

        horizontalFront = new EnhancedServo(hardwareMap, "horizontalFront");
        horizontalBack = new EnhancedServo(hardwareMap, "horizontalBack");
        intake = new EnhancedMotor(hardwareMap, "intake");
        vertical = new EnhancedMotor(hardwareMap, "vertical");
        headlightLeft = new EnhancedServo(hardwareMap, "headlightLeft");
        headlightRight = new EnhancedServo(hardwareMap, "headlightRight");

        vertical.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    @Override
    protected void initStates() {
        setStates(HorizontalFrontState.OPEN, HorizontalBackState.OPEN,
                IntakeState.OFF, VerticalState.OFF, HeadlightState.YELLOW, SortState.UNSORTED);
    }

    @Override
    protected void read() {
        horizontalFrontPosition = get(HorizontalFrontState.class).getValue();
        horizontalBackPosition = get(HorizontalBackState.class).getValue();
        intakePower = get(IntakeState.class).getValue();
        verticalPower = get(VerticalState.class).getValue();

        HeadlightState headlightState = get(HeadlightState.class);
        if (headlightState == HeadlightState.STROBE) {
            updateColorStrobe();
        } else if (headlightState.equals(HeadlightState.CYAN_STROBE) || headlightState.equals(HeadlightState.WHITE_STROBE)) {
            updateBlinkStrobe();
        } else {
            headlightPosition = headlightState.getValue();
        }

        updateColorSensors();
    }

    @Override
    protected void write() {
        horizontalFront.setPosition(horizontalFrontPosition);
        horizontalBack.setPosition(horizontalBackPosition);
        intake.setPower(intakePower);
        vertical.setPower(verticalPower);
        headlightLeft.setPosition(headlightPosition);
        headlightRight.setPosition(headlightPosition);
    }

    private void updateColorSensors() {
        if (!robot.srsHubManager.isReady()) {
            return;
        }

        sensor1 = robot.srsHubManager.getI2CDevice(1, SRSHub.APDS9151.class);
        sensor2 = robot.srsHubManager.getI2CDevice(2, SRSHub.APDS9151.class);
        sensor3 = robot.srsHubManager.getI2CDevice(3, SRSHub.APDS9151.class);

        MagazineState.ArtifactColor color1 = detectColor(sensor1);
        MagazineState.ArtifactColor color2 = detectColor(sensor2);
        MagazineState.ArtifactColor color3 = detectColor(sensor3);

        colorPattern = "" + color1.getSymbol() + color2.getSymbol() + color3.getSymbol();
        currentState = new MagazineState(color1, color2, color3);

        // Determine if the current pattern is exactly one GREEN and two PURPLE
        isValid = currentState.countColor(MagazineState.ArtifactColor.GREEN) == 1 &&
                 currentState.countColor(MagazineState.ArtifactColor.PURPLE) == 2;

//        RobotLog.e("Mag State "+ colorPattern);
//        RobotLog.e("Mag Valid "+ isValid);
    }

    private MagazineState.ArtifactColor detectColor(SRSHub.APDS9151 sensor) {
        if (sensor.disconnected) {
            return MagazineState.ArtifactColor.EMPTY;
        }

        int total = sensor.red + sensor.green + sensor.blue;
        if (total == 0) {
            return MagazineState.ArtifactColor.EMPTY;
        }

        double redRatio = (double) sensor.red / total * 100;
        double greenRatio = (double) sensor.green / total * 100;
        double blueRatio = (double) sensor.blue / total * 100;

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

        if (green > purple && green > 1) {
            return MagazineState.ArtifactColor.GREEN;
        } else if (purple > green && purple > 1) {
            return MagazineState.ArtifactColor.PURPLE;
        } else {
            return MagazineState.ArtifactColor.EMPTY;
        }
    }

    public String getColorPattern() { return colorPattern; }

    public MagazineState.ArtifactColor getSensorColor(int sensorNumber) {
        return currentState.getPosition(sensorNumber);
    }

    public int getBallCount() {
        int balls = 0;
        for (int count = 1; count <= 3; count++) {
            if (!getSensorColor(count).equals(MagazineState.ArtifactColor.EMPTY)) {
                balls++;
            }
        }
        RobotLog.e("balls " + balls);
        return balls;
    }

    public MagazineState getMagazineState() { return currentState; }

    public boolean isFull() { return currentState.isFull(); }

    public boolean isEmpty() { return currentState.isEmpty(); }

    public int countColor(MagazineState.ArtifactColor color) { return currentState.countColor(color); }

    public boolean isValid() {
        RobotLog.e("isValid: " + isValid);
        RobotLog.e("isvalid color pattern: " + colorPattern);
        return isValid; }

    public StorageDecision checkAndStoreBalls(MagazineState targetPattern) {
        MagazineState.ArtifactColor[] current = { getSensorColor(1), getSensorColor(2), getSensorColor(3) };
        MagazineState.ArtifactColor[] target = { targetPattern.getPosition1(), targetPattern.getPosition2(), targetPattern.getPosition3() };

        boolean storeBack = current[0] != target[0];
        boolean storeFront = storeBack ? (current[1] != target[0] || current[2] != target[1])
                                        : (current[1] != target[1] || current[2] != target[2]);

        int shootBeforeOpening = 3 - (storeBack ? 1 : 0) - (storeFront ? 1 : 0);
        int shootAfterOpening = (storeBack ? 1 : 0) + (storeFront ? 1 : 0);

        RobotLog.e("shoot storeBack " + storeBack);
        RobotLog.e("shoot storeFront " + storeFront);

        RobotLog.e("shootBeforeOpening " + shootBeforeOpening);
        RobotLog.e("shootAfterOpening " + shootAfterOpening);

        return new StorageDecision(storeBack, storeFront, shootBeforeOpening, shootAfterOpening);
    }

    public static class StorageDecision {
        public final boolean storeInBack;
        public final boolean storeInFront;
        public final int shootBeforeOpening;
        public final int shootAfterOpening;

        public StorageDecision(boolean storeInBack, boolean storeInFront, int shootBeforeOpening, int shootAfterOpening) {
            this.storeInBack = storeInBack;
            this.storeInFront = storeInFront;
            this.shootBeforeOpening = shootBeforeOpening;
            this.shootAfterOpening = shootAfterOpening;
        }

        @Override
        public String toString() {
            return String.format("StorageDecision[Back=%s, Front=%s, Shoot=%d→%d]", storeInBack, storeInFront, shootBeforeOpening, shootAfterOpening);
        }
    }

    private void updateColorStrobe() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastStrobeUpdate >= STROBE_INTERVAL_MS) {
            lastStrobeUpdate = currentTime;
            if (strobeIncreasing) {
                strobePosition += STROBE_SPEED;
                if (strobePosition >= STROBE_MAX) { strobePosition = STROBE_MAX; strobeIncreasing = false; }
            } else {
                strobePosition -= STROBE_SPEED;
                if (strobePosition <= STROBE_MIN) { strobePosition = STROBE_MIN; strobeIncreasing = true; }
            }
            headlightPosition = strobePosition;
        }
    }

    private void updateBlinkStrobe() {
        long currentTime = System.currentTimeMillis();
        if (Math.sin((double) currentTime / 1000 * 2 * Math.PI) > 0) {
            headlightPosition = HeadlightState.OFF.getValue();
        } else {
            headlightPosition = get(HeadlightState.class).getValue();
        }
    }

    @Override
    protected void onTelemetry() {
        if (magazineTelemetry.TOGGLE) {
            if (magazineTelemetry.intake) {
                logDashboard("Intake State", get(IntakeState.class));
                logDashboard("Intake Power", String.format("%.3f", intakePower));
            }
            if (magazineTelemetry.vertical) {
                logDashboard("Vertical State", get(VerticalState.class));
                logDashboard("Vertical Power", String.format("%.3f", verticalPower));
            }
            if (magazineTelemetry.servos) {
                logDashboard("Horizontal Front State", get(HorizontalFrontState.class));
                logDashboard("Horizontal Front Position", String.format("%.3f", horizontalFrontPosition));
                logDashboard("Horizontal Back State", get(HorizontalBackState.class));
                logDashboard("Horizontal Back Position", String.format("%.3f", horizontalBackPosition));
            }
            if (magazineTelemetry.headlights) {
                logDashboard("Headlight State", get(HeadlightState.class));
                logDashboard("Headlight Position", String.format("%.3f", headlightPosition));
            }
            if (magazineTelemetry.current) {
                logDashboard("Intake Current (A)", String.format("%.2f", intake.getCurrent(CurrentUnit.AMPS)));
                logDashboard("Vertical Current (A)", String.format("%.2f", vertical.getCurrent(CurrentUnit.AMPS)));
            }
            if (magazineTelemetry.colorSensors) {
                logDashboard("Color Pattern", colorPattern);
                logDashboard("Magazine State", currentState);
            }
        }
    }
}
