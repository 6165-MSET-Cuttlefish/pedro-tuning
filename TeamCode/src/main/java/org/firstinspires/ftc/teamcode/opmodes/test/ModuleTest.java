package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.architecture.auto.FieldPose.ColorPose;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.OpModeEx;
import org.firstinspires.ftc.teamcode.lib.State;
import org.firstinspires.ftc.teamcode.modules.Magazine;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;

@Config
@TeleOp(name = "Module Test", group = "Test")
public class ModuleTest extends OpModeEx {
    // Dashboard-exposed fields
    public static String flywheelSelection = Shooter.FlywheelState.OFF.name();
    public static double flywheelValue = Shooter.FlywheelState.OFF.getValue();

    public static String hoodSelection = Shooter.HoodState.BOTTOM.name();
    public static double hoodValue = Shooter.HoodState.BOTTOM.getValue();

    public static String turretSelection = Turret.TurretState.CENTER.name();
    public static double turretValue = Turret.TurretState.CENTER.getValue();

    public static String horizontalFrontSelection = Magazine.HorizontalFrontState.OPEN.name();
    public static double horizontalFrontValue = Magazine.HorizontalFrontState.OPEN.getValue();

    public static String horizontalBackSelection = Magazine.HorizontalBackState.OPEN.name();
    public static double horizontalBackValue = Magazine.HorizontalBackState.OPEN.getValue();

    public static String verticalSelection = Magazine.VerticalState.OFF.name();
    public static double verticalValue = Magazine.VerticalState.OFF.getValue();

    public static String intakeSelection = Magazine.IntakeState.OFF.name();
    public static double intakeValue = Magazine.IntakeState.OFF.getValue();


    public static String headlightSelection = Magazine.HeadlightState.YELLOW.name();
    public static double headlightValue = Magazine.HeadlightState.YELLOW.getValue();

    // Currently active enum constants (for Dashboard reference)
    public static Shooter.FlywheelState flywheelActive = Shooter.FlywheelState.OFF;
    public static Shooter.HoodState hoodActive = Shooter.HoodState.BOTTOM;
    public static Turret.TurretState turretActive = Turret.TurretState.CENTER;
    public static Magazine.HorizontalFrontState horizontalFrontActive = Magazine.HorizontalFrontState.OPEN;
    public static Magazine.HorizontalBackState horizontalBackActive = Magazine.HorizontalBackState.OPEN;
    public static Magazine.VerticalState verticalActive = Magazine.VerticalState.OFF;
    public static Magazine.IntakeState intakeActive = Magazine.IntakeState.OFF;
    public static Magazine.HeadlightState headlightActive = Magazine.HeadlightState.YELLOW;

    // Track previous values to detect value-based selection changes
    private static double previousFlywheelValue = Shooter.FlywheelState.OFF.getValue();
    private static double previousHoodValue = Shooter.HoodState.BOTTOM.getValue();
    private static double previousTurretValue = Turret.TurretState.CENTER.getValue();
    private static double previousHorizontalFrontValue = Magazine.HorizontalFrontState.OPEN.getValue();
    private static double previousHorizontalBackValue = Magazine.HorizontalBackState.OPEN.getValue();
    private static double previousVerticalValue = Magazine.VerticalState.OFF.getValue();
    private static double previousIntakeValue = Magazine.IntakeState.OFF.getValue();
    private static double previousHeadlightValue = Magazine.HeadlightState.YELLOW.getValue();

    // Track previous selections to detect when selection changes
    private static String previousFlywheelSelection = Shooter.FlywheelState.OFF.name();
    private static String previousHoodSelection = Shooter.HoodState.BOTTOM.name();
    private static String previousTurretSelection = Turret.TurretState.CENTER.name();
    private static String previousHorizontalFrontSelection = Magazine.HorizontalFrontState.OPEN.name();
    private static String previousHorizontalBackSelection = Magazine.HorizontalBackState.OPEN.name();
    private static String previousVerticalSelection = Magazine.VerticalState.OFF.name();
    private static String previousIntakeSelection = Magazine.IntakeState.OFF.name();
    private static String previousHeadlightSelection = Magazine.HeadlightState.YELLOW.name();

    private static final double VALUE_EPSILON = 1e-6;

    private final Pose setupPose = ColorPose(87, 14.7, Math.toRadians(90));

    // Helper interface for state handling
    @FunctionalInterface
    private interface StateUpdater<S extends Enum<S> & State> {
        void update(S state);
    }

    @Override
    public void initialize() {
        robot.follower.setPose(setupPose);

        // Reset all dashboard fields
        flywheelSelection = Shooter.FlywheelState.OFF.name();
        flywheelValue = Shooter.FlywheelState.OFF.getValue();
        hoodSelection = Shooter.HoodState.BOTTOM.name();
        hoodValue = Shooter.HoodState.BOTTOM.getValue();
        turretSelection = Turret.TurretState.CENTER.name();
        turretValue = Turret.TurretState.CENTER.getValue();
        horizontalFrontSelection = Magazine.HorizontalFrontState.OPEN.name();
        horizontalFrontValue = Magazine.HorizontalFrontState.OPEN.getValue();
        horizontalBackSelection = Magazine.HorizontalBackState.OPEN.name();
        horizontalBackValue = Magazine.HorizontalBackState.OPEN.getValue();
        verticalSelection = Magazine.VerticalState.OFF.name();
        verticalValue = Magazine.VerticalState.OFF.getValue();
        intakeSelection = Magazine.IntakeState.OFF.name();
        intakeValue = Magazine.IntakeState.OFF.getValue();
        headlightSelection = Magazine.HeadlightState.YELLOW.name();
        headlightValue = Magazine.HeadlightState.YELLOW.getValue();

        // Reset active fields
        flywheelActive = Shooter.FlywheelState.OFF;
        hoodActive = Shooter.HoodState.BOTTOM;
        turretActive = Turret.TurretState.CENTER;
        horizontalFrontActive = Magazine.HorizontalFrontState.OPEN;
        horizontalBackActive = Magazine.HorizontalBackState.OPEN;
        verticalActive = Magazine.VerticalState.OFF;
        intakeActive = Magazine.IntakeState.OFF;
        headlightActive = Magazine.HeadlightState.YELLOW;

        // Reset previous selection tracking
        previousFlywheelSelection = Shooter.FlywheelState.OFF.name();
        previousHoodSelection = Shooter.HoodState.BOTTOM.name();
        previousTurretSelection = Turret.TurretState.CENTER.name();
        previousHorizontalFrontSelection = Magazine.HorizontalFrontState.OPEN.name();
        previousHorizontalBackSelection = Magazine.HorizontalBackState.OPEN.name();
        previousVerticalSelection = Magazine.VerticalState.OFF.name();
        previousIntakeSelection = Magazine.IntakeState.OFF.name();
        previousHeadlightSelection = Magazine.HeadlightState.YELLOW.name();

        // Reset previous value tracking
        previousFlywheelValue = Shooter.FlywheelState.OFF.getValue();
        previousHoodValue = Shooter.HoodState.BOTTOM.getValue();
        previousTurretValue = Turret.TurretState.CENTER.getValue();
        previousHorizontalFrontValue = Magazine.HorizontalFrontState.OPEN.getValue();
        previousHorizontalBackValue = Magazine.HorizontalBackState.OPEN.getValue();
        previousVerticalValue = Magazine.VerticalState.OFF.getValue();
        previousIntakeValue = Magazine.IntakeState.OFF.getValue();
        previousHeadlightValue = Magazine.HeadlightState.YELLOW.getValue();
    }

    @Override
    public void initializeLoop() {}

    @Override
    public void primaryLoop() {
        handleControllerInput();
        
        processFlywheelState();
        processHoodState();
        processTurretState();
        processHorizontalFrontState();
        processHorizontalBackState();
        processVerticalState();
        processIntakeState();
        processHeadlightState();
    }

    private void handleControllerInput() {
        // Right Bumper: Toggle VerticalState between UP and DOWN
        if (gamepad1.rightBumperWasPressed()) {
            Magazine.VerticalState current = Enum.valueOf(Magazine.VerticalState.class, verticalSelection);
            if (current == Magazine.VerticalState.ON) {
                verticalSelection = Magazine.VerticalState.OFF.name();
            } else if (current == Magazine.VerticalState.OFF) {
                verticalSelection = Magazine.VerticalState.ON.name();
            }
        }

        // A Button: Toggle IntakeState between FORWARD and OFF
        if (gamepad1.aWasPressed()) {
            Magazine.IntakeState current = Enum.valueOf(Magazine.IntakeState.class, intakeSelection);
            if (current == Magazine.IntakeState.FORWARD) {
                intakeSelection = Magazine.IntakeState.OFF.name();
            } else {
                intakeSelection = Magazine.IntakeState.FORWARD.name();
            }
        }
    }

    private void processFlywheelState() {
        // If selection changed, update value from enum
        if (!flywheelSelection.equals(previousFlywheelSelection)) {
            Shooter.FlywheelState selected = Enum.valueOf(Shooter.FlywheelState.class, flywheelSelection);
            flywheelValue = selected.getValue();
            previousFlywheelSelection = flywheelSelection;
        }
        // If value changed, update enum constant
        if (!almostEqual(flywheelValue, previousFlywheelValue)) {
            Shooter.FlywheelState selected = Enum.valueOf(Shooter.FlywheelState.class, flywheelSelection);
            selected.setValue(flywheelValue);
            previousFlywheelValue = flywheelValue;
        }
        processState("flywheel", flywheelSelection, flywheelValue, Shooter.FlywheelState.class,
                state -> {
                    state.apply();
                    flywheelActive = state;
                    robot.shooter.targetVelocityRPM = state.getValue();
                });
    }

    private void processHoodState() {
        // If selection changed, update value from enum
        if (!hoodSelection.equals(previousHoodSelection)) {
            Shooter.HoodState selected = Enum.valueOf(Shooter.HoodState.class, hoodSelection);
            hoodValue = selected.getValue();
            previousHoodSelection = hoodSelection;
        }
        // If value changed, update enum constant
        if (!almostEqual(hoodValue, previousHoodValue)) {
            Shooter.HoodState selected = Enum.valueOf(Shooter.HoodState.class, hoodSelection);
            selected.setValue(hoodValue);
            previousHoodValue = hoodValue;
        }
        processState("hood", hoodSelection, hoodValue, Shooter.HoodState.class,
                state -> {
                    state.apply();
                    hoodActive = state;
                });
    }

    private void processTurretState() {
        // If selection changed, update value from enum
        if (!turretSelection.equals(previousTurretSelection)) {
            Turret.TurretState selected = Enum.valueOf(Turret.TurretState.class, turretSelection);
            turretValue = selected.getValue();
            previousTurretSelection = turretSelection;
        }
        // If value changed, update enum constant
        if (!almostEqual(turretValue, previousTurretValue)) {
            Turret.TurretState selected = Enum.valueOf(Turret.TurretState.class, turretSelection);
            selected.setValue(turretValue);
            previousTurretValue = turretValue;
        }
        processState("turret", turretSelection, turretValue, Turret.TurretState.class,
                state -> {
                    state.apply();
                    turretActive = state;
                });
    }

    private void processHorizontalFrontState() {
        // If selection changed, update value from enum
        if (!horizontalFrontSelection.equals(previousHorizontalFrontSelection)) {
            Magazine.HorizontalFrontState selected = Enum.valueOf(Magazine.HorizontalFrontState.class, horizontalFrontSelection);
            horizontalFrontValue = selected.getValue();
            previousHorizontalFrontSelection = horizontalFrontSelection;
        }
        // If value changed, update enum constant
        if (!almostEqual(horizontalFrontValue, previousHorizontalFrontValue)) {
            Magazine.HorizontalFrontState selected = Enum.valueOf(Magazine.HorizontalFrontState.class, horizontalFrontSelection);
            selected.setValue(horizontalFrontValue);
            previousHorizontalFrontValue = horizontalFrontValue;
        }
        processState("horizontalFront", horizontalFrontSelection, horizontalFrontValue, 
                Magazine.HorizontalFrontState.class,
                state -> {
                    state.apply();
                    horizontalFrontActive = state;
                });
    }

    private void processHorizontalBackState() {
        // If selection changed, update value from enum
        if (!horizontalBackSelection.equals(previousHorizontalBackSelection)) {
            Magazine.HorizontalBackState selected = Enum.valueOf(Magazine.HorizontalBackState.class, horizontalBackSelection);
            horizontalBackValue = selected.getValue();
            previousHorizontalBackSelection = horizontalBackSelection;
        }
        // If value changed, update enum constant
        if (!almostEqual(horizontalBackValue, previousHorizontalBackValue)) {
            Magazine.HorizontalBackState selected = Enum.valueOf(Magazine.HorizontalBackState.class, horizontalBackSelection);
            selected.setValue(horizontalBackValue);
            previousHorizontalBackValue = horizontalBackValue;
        }
        processState("horizontalBack", horizontalBackSelection, horizontalBackValue, 
                Magazine.HorizontalBackState.class,
                state -> {
                    state.apply();
                    horizontalBackActive = state;
                });
    }

    private void processVerticalState() {
        // If selection changed, update value from enum
        if (!verticalSelection.equals(previousVerticalSelection)) {
            Magazine.VerticalState selected = Enum.valueOf(Magazine.VerticalState.class, verticalSelection);
            verticalValue = selected.getValue();
            previousVerticalSelection = verticalSelection;
        }
        // If value changed, update enum constant
        if (!almostEqual(verticalValue, previousVerticalValue)) {
            Magazine.VerticalState selected = Enum.valueOf(Magazine.VerticalState.class, verticalSelection);
            selected.setValue(verticalValue);
            previousVerticalValue = verticalValue;
        }
        processState("vertical", verticalSelection, verticalValue, Magazine.VerticalState.class,
                state -> {
                    state.apply();
                    verticalActive = state;
                });
    }

    private void processIntakeState() {
        // If selection changed, update value from enum
        if (!intakeSelection.equals(previousIntakeSelection)) {
            Magazine.IntakeState selected = Enum.valueOf(Magazine.IntakeState.class, intakeSelection);
            intakeValue = selected.getValue();
            previousIntakeSelection = intakeSelection;
        }
        // If value changed, update enum constant
        if (!almostEqual(intakeValue, previousIntakeValue)) {
            Magazine.IntakeState selected = Enum.valueOf(Magazine.IntakeState.class, intakeSelection);
            selected.setValue(intakeValue);
            previousIntakeValue = intakeValue;
        }
        processState("intake", intakeSelection, intakeValue, Magazine.IntakeState.class,
                state -> {
                    state.apply();
                    intakeActive = state;
                });
    }

    private void processHeadlightState() {
        // If selection changed, update value from enum
        if (!headlightSelection.equals(previousHeadlightSelection)) {
            Magazine.HeadlightState selected = Enum.valueOf(Magazine.HeadlightState.class, headlightSelection);
            headlightValue = selected.getValue();
            previousHeadlightSelection = headlightSelection;
        }
        // If value changed, update enum constant
        if (!almostEqual(headlightValue, previousHeadlightValue)) {
            Magazine.HeadlightState selected = Enum.valueOf(Magazine.HeadlightState.class, headlightSelection);
            selected.setValue(headlightValue);
            previousHeadlightValue = headlightValue;
        }
        processState("headlight", headlightSelection, headlightValue, Magazine.HeadlightState.class,
                state -> {
                    state.apply();
                    headlightActive = state;
                });
    }

    /**
     * Generic method to process state changes from dashboard.
     */
    private <S extends Enum<S> & State> void processState(
            String stateName,
            String enumSelection,
            double stateValue,
            Class<S> stateClass,
            StateUpdater<S> updater) {
        try {
            S selected = Enum.valueOf(stateClass, enumSelection);
            updater.update(selected);
            robot.telemetry.addData(stateName + "State", selected.name());
            robot.telemetry.addData(stateName + "Value", stateValue);
        } catch (Exception e) {
            robot.telemetry.addData(stateName + "_error", "Error: " + e.getMessage());
        }
    }

    /**
     * Checks if two double values are approximately equal.
     */
    private static boolean almostEqual(double a, double b) {
        return Math.abs(a - b) < VALUE_EPSILON;
    }
}
