package org.firstinspires.ftc.teamcode.opmodes.tele;

import static org.firstinspires.ftc.teamcode.modules.MagazineState.ArtifactColor.GREEN;
import static org.firstinspires.ftc.teamcode.modules.MagazineState.ArtifactColor.PURPLE;
import static org.firstinspires.ftc.teamcode.modules.Turret.turretOffset;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.layers.ACTION_LAYER;
import org.firstinspires.ftc.teamcode.architecture.layers.LayeredGamepad;
import org.firstinspires.ftc.teamcode.architecture.layers.MapLayeringSystem;
import org.firstinspires.ftc.teamcode.architecture.layers.suppliers.CustomGamepad;
import org.firstinspires.ftc.teamcode.architecture.layers.suppliers.EnhancedBooleanSupplier;
import org.firstinspires.ftc.teamcode.architecture.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.core.Context;
import org.firstinspires.ftc.teamcode.core.OpModeEx;
import org.firstinspires.ftc.teamcode.modules.Endgame;
import org.firstinspires.ftc.teamcode.modules.Magazine;
import org.firstinspires.ftc.teamcode.modules.MagazineState;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;

import java.util.HashMap;
import java.util.Map;

@Config
@TeleOp(name = "Tele", group = "A")
public class Tele extends OpModeEx {
    private ACTION_LAYER actionLayer = ACTION_LAYER.TELE;
    private LayeredGamepad<ACTION_LAYER> driver1Gamepad;
    private LayeredGamepad<ACTION_LAYER> driver2Gamepad;

    private boolean ninjaMode = false;
    public static double ninjaModeMultiplier = 0.5;
    private boolean intakeOn = false;
    private boolean autoSorting = false;

    private EnhancedBooleanSupplier driver1LeftTrigger;
    private EnhancedBooleanSupplier driver1RightTrigger;
    private EnhancedBooleanSupplier driver2LeftTrigger;
    private EnhancedBooleanSupplier driver2RightTrigger;

    private EnhancedBooleanSupplier driver2RightStickUp;
    private EnhancedBooleanSupplier driver2RightStickDown;
    private EnhancedBooleanSupplier driver2RightStickRight;
    private EnhancedBooleanSupplier driver2RightStickLeft;

    private EnhancedBooleanSupplier driver2LeftStickUp;
    private EnhancedBooleanSupplier driver2LeftStickDown;
    private EnhancedBooleanSupplier driver2LeftStickRight;
    private EnhancedBooleanSupplier driver2LeftStickLeft;

    private Magazine.IntakeState oldIntake;

    Magazine.VerticalState oldVerticalState;

    @Override
    public void initialize() {
        initializeGamepadLayers();
    }

    @Override
    public void initializeLoop() {
        handleLayers();

        if (driver1Gamepad.Y().wasJustPressed()) {
            robot.follower.setPose(new Pose(72, 72, Math.PI/2));
        }

        robot.endgame.leftInitialEncoder.zero();
        robot.endgame.rightInitialEncoder.zero();
    }

    @Override
    public void onStart() {
        robot.drivetrain.getFl().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.drivetrain.getFr().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.drivetrain.getBl().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.drivetrain.getBr().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void primaryLoop() {
        handleLayers();

        if (!robot.endgame.get(Endgame.FullLiftState.class).equals(Endgame.FullLiftState.FULL_LIFT)) {
            handleDrivetrainMovement();
        }

        FieldVisualization.drawRobot(robot.follower.getPose());
        FieldVisualization.drawPoseHistory(robot.follower.getPoseHistory());
    }

    private void switchToLayer(ACTION_LAYER newLayer) {
        actionLayer = newLayer;
        driver1Gamepad.setLayer(newLayer);
        driver2Gamepad.setLayer(newLayer);
    }

    private void handleLayers() {
        driver1Gamepad.invalidateAll();
        driver2Gamepad.invalidateAll();

        driver1LeftTrigger.invalidate();
        driver1RightTrigger.invalidate();
        driver2LeftTrigger.invalidate();
        driver2RightTrigger.invalidate();

        driver2RightStickUp.invalidate();
        driver2RightStickDown.invalidate();
        driver2RightStickRight.invalidate();
        driver2RightStickLeft.invalidate();

        robot.telemetry.addData("Active Layer", actionLayer);

        switch (actionLayer) {
            case TELE:
                teleLayer();
                break;
            case ENDGAME:
                endgameLayer();
                break;
        }
    }

    private void initializeGamepadLayers() {
        CustomGamepad teleGamepad1 = new CustomGamepad(gamepad1);
        CustomGamepad endgameGamepad1 = new CustomGamepad(gamepad1);
        CustomGamepad teleGamepad2 = new CustomGamepad(gamepad2);
        CustomGamepad endgameGamepad2 = new CustomGamepad(gamepad2);

        Map<ACTION_LAYER, CustomGamepad> driver1Layers =
                new HashMap<ACTION_LAYER, CustomGamepad>() {
                    {
                        put(ACTION_LAYER.TELE, teleGamepad1);
                        put(ACTION_LAYER.ENDGAME, endgameGamepad1);
                    }
                };

        Map<ACTION_LAYER, CustomGamepad> driver2Layers =
                new HashMap<ACTION_LAYER, CustomGamepad>() {
                    {
                        put(ACTION_LAYER.TELE, teleGamepad2);
                        put(ACTION_LAYER.ENDGAME, endgameGamepad2);
                    }
                };

        MapLayeringSystem<ACTION_LAYER> layeringSystem1 =
                new MapLayeringSystem<>(ACTION_LAYER.TELE, driver1Layers);
        MapLayeringSystem<ACTION_LAYER> layeringSystem2 =
                new MapLayeringSystem<>(ACTION_LAYER.TELE, driver2Layers);

        driver1Gamepad = new LayeredGamepad<>(layeringSystem1);
        driver2Gamepad = new LayeredGamepad<>(layeringSystem2);

        driver1Gamepad.setLayer(ACTION_LAYER.TELE);
        driver2Gamepad.setLayer(ACTION_LAYER.TELE);

        driver1LeftTrigger = driver1Gamepad.LT().greaterThan(0.5);
        driver1RightTrigger = driver1Gamepad.RT().greaterThan(0.5);
        driver2LeftTrigger = driver2Gamepad.LT().greaterThan(0.5);
        driver2RightTrigger = driver2Gamepad.RT().greaterThan(0.5);

        driver2RightStickUp = driver2Gamepad.getRightStickY().lessThan(-0.5);
        driver2RightStickDown = driver2Gamepad.getRightStickY().greaterThan(0.5);
        driver2RightStickRight = driver2Gamepad.getRightStickX().greaterThan(0.5);
        driver2RightStickLeft = driver2Gamepad.getRightStickX().lessThan(-0.5);

        driver2LeftStickUp = driver2Gamepad.getLeftStickY().lessThan(-0.5);
        driver2LeftStickDown = driver2Gamepad.getLeftStickY().greaterThan(0.5);
        driver2LeftStickRight = driver2Gamepad.getLeftStickX().greaterThan(0.5);
        driver2LeftStickLeft = driver2Gamepad.getLeftStickX().lessThan(-0.5);
    }

    private void teleLayer() {
        if (driver2Gamepad.LSB().wasJustPressed()) {
            autoSorting = !autoSorting;
        }
        if (driver2Gamepad.RSB().wasJustPressed()) {
            robot.actions.wiggleBackHzPusher().run();
        }

        if (driver1Gamepad.B().wasJustPressed()) {
            Shooter.FlywheelState.IDLE.apply();
            Shooter.HoodState.BOTTOM.apply();
        }

        if (driver2Gamepad.DPAD_UP().wasJustPressed()) {
            if (robot.magazine.get(Magazine.IntakeState.class).equals(Magazine.IntakeState.FORWARD)) {
                Magazine.IntakeState.OFF.apply();
            } else {
                Magazine.IntakeState.FORWARD.apply();
            }
        } else if (driver2Gamepad.DPAD_RIGHT().wasJustPressed()) {
            if (!autoSorting) {
                (robot.magazine.get(Magazine.HorizontalFrontState.class)
                                        .equals(Magazine.HorizontalFrontState.OPEN)
                                ? Magazine.HorizontalFrontState.STORED
                                : Magazine.HorizontalFrontState.OPEN).apply();
            } else {
                Context.motif = new MagazineState(PURPLE, GREEN, PURPLE);
                robot.actions.sortMagazine().run();
            }
        } else if (driver2Gamepad.DPAD_DOWN().wasJustPressed()) {
            if (!autoSorting) {
            } else {
                Context.motif = new MagazineState(PURPLE, PURPLE, GREEN);
                robot.actions.sortMagazine().run();
            }
        } else if (driver2Gamepad.DPAD_LEFT().wasJustPressed()) {
            if (!autoSorting) {
                (robot.magazine.get(Magazine.HorizontalBackState.class)
                                        .equals(Magazine.HorizontalBackState.OPEN)
                                ? Magazine.HorizontalBackState.STORED
                                : Magazine.HorizontalBackState.OPEN).apply();
            } else {
                Context.motif = new MagazineState(GREEN, PURPLE, PURPLE);
                robot.actions.sortMagazine().run();
            }
        }

        if (driver2Gamepad.X().wasJustPressed()) {
            Shooter.FlywheelState.IDLE.apply();
            Shooter.HoodState.BOTTOM.apply();
        } else if (driver2Gamepad.Y().wasJustPressed()) {
            Shooter.FlywheelState.OFF.apply();
            Shooter.HoodState.BOTTOM.apply();
        } else if (driver2Gamepad.A().wasJustPressed()) {
            Shooter.FlywheelState.PID.apply();
            Shooter.HoodState.PID.apply();
        } else if (driver2Gamepad.B().wasJustPressed()) {
            Shooter.FlywheelState.PID.apply();
            Shooter.HoodState.PID.apply();
        }

        if (driver2RightStickUp.wasJustPressed()) {
            robot.shooter.velocityOffset += 50;
        } else if (driver2RightStickDown.wasJustPressed()) {
            robot.shooter.velocityOffset -= 50;
        }

        if (driver2RightStickRight.wasJustPressed()) {
            robot.shooter.hoodOffset += 0.01;
        } else if (driver2RightStickLeft.wasJustPressed()) {
            robot.shooter.hoodOffset -= 0.01;
        }

        if (driver2Gamepad.getLeftStickX().getState() > 0.5) {
            turretOffset -= 1;
        }
        if (driver2Gamepad.getLeftStickX().getState() < -0.5) {
            turretOffset += 1;
        }

        if (gamepad1.right_bumper) {
            Magazine.VerticalState.HALF_DOWN.apply();
        } else if (gamepad1.right_trigger_pressed) {
            Magazine.VerticalState.ON.apply();
        } else {
            Magazine.VerticalState.OFF.apply();
        }

        if (gamepad2.right_trigger_pressed) {
            oldIntake = robot.magazine.get(Magazine.IntakeState.class);
            Magazine.IntakeState.HALF.apply();
        } else if (gamepad2.left_trigger_pressed) {
            oldIntake = robot.magazine.get(Magazine.IntakeState.class);
            double analogValue = gamepad2.left_trigger;
            Magazine.IntakeState.MANUAL.setValue(-analogValue);
            Magazine.IntakeState.MANUAL.apply();
        } if (gamepad2.leftTriggerWasReleased() || gamepad2.rightTriggerWasReleased()) {
            Magazine.IntakeState.OFF.apply();
        }

        if (driver2Gamepad.LB().wasJustPressed()) {
            (robot.turret.get(Turret.TurretState.class).equals(Turret.TurretState.AUTOAIM)
                            ? Turret.TurretState.HOLD
                            : Turret.TurretState.AUTOAIM).apply();
        }

//        if (driver2Gamepad.RB().wasJustPressed()) {
//            switchToLayer(ACTION_LAYER.ENDGAME);
//        }

        if (driver1LeftTrigger.wasJustPressed()) {
            ninjaMode = !ninjaMode;
        }

        if (driver1Gamepad.DPAD_LEFT().wasJustPressed()) {
            robot.follower.setPose(new Pose(10, 9, Math.toRadians(180)));
        }
        if (driver1Gamepad.DPAD_RIGHT().wasJustPressed()) {
            robot.follower.setPose(new Pose(144 - 10, 9, Math.toRadians(0)));
        }
        if (driver1Gamepad.DPAD_UP().wasJustPressed()) {
            robot.follower.setPose(new Pose(72, 10, Math.toRadians(90)));
        }
        if (driver1Gamepad.DPAD_DOWN().wasJustPressed()) {
            robot.follower.setPose(new Pose(72, 144 - 10, Math.toRadians(90)));
        }

        if (robot.shooter.isAtTargetVelocity()) {
            Magazine.headlightPosition = Magazine.HeadlightState.CYAN.getValue();
            if (robot.magazine.isFull()) {
                Magazine.HeadlightState.GREEN.apply();
            } else {
                Magazine.HeadlightState.CYAN.apply();
            }
        } else {
            if (robot.magazine.isFull()) {
                Magazine.HeadlightState.WHITE_STROBE.apply();
            } else {
                Magazine.HeadlightState.WHITE.apply();
            }
        }

        if (driver1Gamepad.RB().isDown() && driver1Gamepad.LB().isDown()
                && driver1RightTrigger.isDown() && driver1LeftTrigger.isDown()) {

            if (driver1Gamepad.B().isDown()) {
                Context.allianceColor = AllianceColor.RED;
            } else if (driver1Gamepad.X().isDown()) {
                Context.allianceColor = AllianceColor.BLUE;
            }

        }

        robot.telemetry.addData("Sorting?", autoSorting ? "Yes" : "No");
    }

    private void endgameLayer() {
        if (driver2Gamepad.X().wasJustPressed()) {
            robot.actions.endgameSequence().run();
        }
//        if (driver2Gamepad.RB().wasJustPressed()) {
//            switchToLayer(ACTION_LAYER.TELE);
//        }
    }

    private void handleDrivetrainMovement() {
        double y = -driver1Gamepad.getLeftStickY().getState();
        double x = driver1Gamepad.getLeftStickX().getState();
        double rx = driver1Gamepad.getRightStickX().getState();

        if (ninjaMode) {
            y *= ninjaModeMultiplier;
            x *= ninjaModeMultiplier;
            rx *= ninjaModeMultiplier;
        }

        robot.drivetrain.setMecanumTargets(y, x, rx, false);

        robot.telemetry.addData("Slow Mode", ninjaMode);
    }
}
