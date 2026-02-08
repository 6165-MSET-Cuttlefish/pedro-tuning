package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.lib.Action;
import org.firstinspires.ftc.teamcode.lib.Actions;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.Endgame;
import org.firstinspires.ftc.teamcode.modules.Magazine;
import org.firstinspires.ftc.teamcode.modules.Shooter;

public class RobotActions {
    private final Robot robot;

    public RobotActions(Robot robot) {
        this.robot = robot;
    }

    public Action prepareShooterFar() {
        return Actions.set(Shooter.FlywheelState.PID, Shooter.HoodState.PID);
    }

    public Action prepareShooterClose() {
        return Actions.set(Shooter.FlywheelState.PID, Shooter.HoodState.PID);
    }

    public Action intakeUntilFull() {
        return Actions.builder()
                .set(Magazine.IntakeState.FORWARD)
                .waitUntil(() -> robot.magazine.getBallCount() == 3)
                .set(Magazine.IntakeState.IDLE)
                .build();
    }

    public Action shootWhenReady() {
        return Actions.builder()
                .waitUntil(() -> robot.shooter.isAtTargetVelocity())
                .set(Magazine.VerticalState.ON)
                .build();
    }

    public Action shootAll() {

        return Actions.builder()
                .waitUntil(() -> robot.shooter.isAtTargetVelocity())
                .set(Magazine.IntakeState.FORWARD, Magazine.VerticalState.ON)
                .delay(Magazine.shootTime)
                .set(Magazine.VerticalState.HALF_DOWN) // Attempt to push the third ball all the way down before shooting it
                .delay(250)
                .set(Magazine.VerticalState.ON)
                .delay(300)
                .set(Magazine.VerticalState.HALF_DOWN)
                .delay(250)
                .set(Magazine.VerticalState.ON)
                .delay(300)
                .set(Magazine.VerticalState.OFF)
                .build();
    }

    public Action sortMagazine() {
        return Actions.builder()
                .run(() -> {
                    Magazine.StorageDecision decision =
                            robot.magazine.checkAndStoreBalls(Context.motif);

                    Context.shootCountBeforeOpening = decision.shootBeforeOpening;
                    Context.shootCountAfterOpening = decision.shootAfterOpening;
                    Context.usedFrontStorage = decision.storeInFront;

                    (decision.storeInBack
                            ? Magazine.HorizontalBackState.STORED
                            : Magazine.HorizontalBackState.OPEN).apply();
                    (decision.storeInFront
                            ? Magazine.HorizontalFrontState.STORED
                            : Magazine.HorizontalFrontState.OPEN).apply();
                })
                .delay(Magazine.horizontalTime)
                // if both stored then delay hz + wait for vertical to come down + delay transfer1
                // if frontstored then delay hz + delay transfer1
                .set(Magazine.SortState.PREPARED)
                .build();
    }

    public Action sortMagazineIfValid() {
        return Actions.builder()
                .ifThen(() -> robot.magazine.isValid(), sortMagazine())
                .build();
    }

    public Action shootSorted() {
        Action sortedSequence = Actions.builder()
                .set(Magazine.IntakeState.FORWARD)
//                .delay(Magazine.transferTime)
                .set(Magazine.VerticalState.ON)
                .set(Magazine.HorizontalBackState.OPEN, Magazine.HorizontalFrontState.OPEN)
                .delay(Magazine.horizontalTime)
                .ifThen(
                        () -> Context.usedFrontStorage && Context.shootCountAfterOpening > 0,
                        Actions.delay(Magazine.transferTime))
                .set(Magazine.VerticalState.ON)
                .set(Magazine.SortState.UNSORTED)
                .build();

        return Actions.builder()
                .ifElse(
                        () -> robot.magazine.get(Magazine.SortState.class).equals(Magazine.SortState.PREPARED),
                        sortedSequence,
                        shootAll())
                .build();
    }

    public Action endgameSequence() {
        return Actions.builder()
                .set(Endgame.InitialState.LIFT, Endgame.InitialState.LIFT, Drivetrain.DriveState.ENDGAME)
                .run(() -> RobotLog.e("set initialstate"))
                .waitUntil(() -> robot.endgame.initialLiftComplete()) //tune threshold
                .run(() -> RobotLog.e("initial lift complete"))
                .run(() -> {
                    robot.drivetrain.getFl().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.drivetrain.getFr().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.drivetrain.getFl().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.drivetrain.getFr().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                })
                .set(Endgame.LeftPtoState.DOWN, Endgame.RightPtoState.DOWN)
                .delay(1000)
                .run(() -> RobotLog.e("initial pto down"))
                .run(() -> robot.endgameDecelTimer.reset())
                .set(Endgame.InitialState.DISABLED, Endgame.InitialState.DISABLED)
                .run(() -> RobotLog.e("initial lift servos disabled"))
                .set(Endgame.FullLiftState.FULL_LIFT)
                .build();
    }

    public Action wiggleBackHzPusher() {
        return Actions.builder()
                .repeat(onePeriod(), 10)
                .build();
    }
    private Action onePeriod() {
        return Actions.builder()
                .set(Magazine.HorizontalBackState.SLIGHT_CLOSED)
                .delay(125)
                .set(Magazine.HorizontalBackState.OPEN)
                .delay(125)
                .build();
    }

}
