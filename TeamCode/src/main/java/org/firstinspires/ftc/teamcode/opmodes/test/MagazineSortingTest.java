package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Context;
import org.firstinspires.ftc.teamcode.core.OpModeEx;
import org.firstinspires.ftc.teamcode.lib.Actions;
import org.firstinspires.ftc.teamcode.modules.Magazine;
import org.firstinspires.ftc.teamcode.modules.MagazineState;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;

@TeleOp(name = "Magazine Sorting Test", group = "Test")
public class MagazineSortingTest extends OpModeEx {
    private boolean intakeRunning = false;
    private boolean transferRunning = false;

    @Override
    protected void initialize() {
        Context.motif = new MagazineState(MagazineState.ArtifactColor.GREEN,
                MagazineState.ArtifactColor.PURPLE, MagazineState.ArtifactColor.PURPLE);
    }

    @Override
    protected void primaryLoop() {
        Turret.TurretState.LEFT.apply();

        handleTargetPatternSelection();
        handleSortingActions();
        handleManualControls();
        handleIntakeTransferControls();
        updateCustomTelemetry();
    }

    private void handleTargetPatternSelection() {
        if (gamepad1.dpad_up && !gamepad1.ps) {
            Context.motif = new MagazineState(MagazineState.ArtifactColor.PURPLE,
                    MagazineState.ArtifactColor.PURPLE, MagazineState.ArtifactColor.GREEN);
        }

        if (gamepad1.dpad_down && !gamepad1.ps) {
            Context.motif = new MagazineState(MagazineState.ArtifactColor.PURPLE,
                    MagazineState.ArtifactColor.GREEN, MagazineState.ArtifactColor.PURPLE);
        }

        if (gamepad1.dpad_left && !gamepad1.ps) {
            Context.motif = new MagazineState(MagazineState.ArtifactColor.GREEN,
                    MagazineState.ArtifactColor.PURPLE, MagazineState.ArtifactColor.PURPLE);
        }
    }

    private void handleSortingActions() {
        if (gamepad1.a) {
            robot.actions.sortMagazine().run();
        }

        if (gamepad1.b) {
            Shooter.FlywheelState.IDLE.apply();
            robot.actions.shootSorted().run();
        }
    }

    private void handleManualControls() {
        if (gamepad1.x) {
            Magazine.HorizontalBackState.OPEN.apply();
            Magazine.HorizontalFrontState.OPEN.apply();
        }

        if (gamepad1.y) {
            Magazine.HorizontalBackState.STORED.apply();
            Magazine.HorizontalFrontState.STORED.apply();
        }
    }

    private void handleIntakeTransferControls() {
        if (Actions.isModuleActive(robot.magazine)) {
            intakeRunning = false;
            transferRunning = false;
            return;
        }

        if (gamepad1.left_bumper) {
            intakeRunning = !intakeRunning;
            if (intakeRunning) {
                Magazine.IntakeState.FORWARD.apply();
            } else {
                Magazine.IntakeState.OFF.apply();
            }
        }

        if (gamepad1.left_trigger > 0.1) {
            Magazine.IntakeState.REVERSE.apply();
            intakeRunning = false;
        } else if (!intakeRunning && gamepad1.left_trigger <= 0.1) {
            Magazine.IntakeState.OFF.apply();
        }
    }

    private void updateCustomTelemetry() {
        robot.telemetry.addLine("=== MAGAZINE SORTING TEST ===");
        robot.telemetry.addLine();

        robot.telemetry.addData("Target Pattern", Context.motif.toPattern());
        robot.telemetry.addData("Current Pattern", robot.magazine.getColorPattern());
        robot.telemetry.addLine();

        robot.telemetry.addData("Sensor 1 (Back Compartment)", robot.magazine.getSensorColor(1));
        robot.telemetry.addData("Sensor 2 (Front Compartment)", robot.magazine.getSensorColor(2));
        robot.telemetry.addData("Sensor 3 (Intake)", robot.magazine.getSensorColor(3));
        robot.telemetry.addLine();

        Magazine.StorageDecision decision = robot.magazine.checkAndStoreBalls(Context.motif);
        robot.telemetry.addData("Store in Back", decision.storeInBack ? "YES" : "NO");
        robot.telemetry.addData("Store in Front", decision.storeInFront ? "YES" : "NO");
        robot.telemetry.addLine();

        robot.telemetry.addData(
                "Back Compartment", robot.magazine.get(Magazine.HorizontalBackState.class));
        robot.telemetry.addData(
                "Front Compartment", robot.magazine.get(Magazine.HorizontalFrontState.class));
        robot.telemetry.addData(
                "Vertical Pusher", robot.magazine.get(Magazine.VerticalState.class));
        robot.telemetry.addLine();

        robot.telemetry.addData("Intake", robot.magazine.get(Magazine.IntakeState.class));
        robot.telemetry.addData("Vertical", robot.magazine.get(Magazine.VerticalState.class));
        robot.telemetry.addLine();

        robot.telemetry.addLine("=== CONTROLS ===");
        robot.telemetry.addData("D-pad", "Set target pattern");
        robot.telemetry.addData("A", "Prepare magazine");
        robot.telemetry.addData("B", "Full shoot sequence");
        robot.telemetry.addData("X", "Open compartments");
        robot.telemetry.addData("Y", "Close compartments");
        robot.telemetry.addData("LB", "Toggle intake");
        robot.telemetry.addData("RB", "Toggle transfer");
        robot.telemetry.addData("LT/RT", "Reverse intake/transfer");
    }
}
