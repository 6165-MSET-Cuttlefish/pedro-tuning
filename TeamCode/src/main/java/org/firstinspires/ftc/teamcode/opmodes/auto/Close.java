package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.architecture.auto.FieldPose.ColorPose;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.auto.Auto;
import org.firstinspires.ftc.teamcode.architecture.auto.pathaction.PathActionBuilder;
import org.firstinspires.ftc.teamcode.architecture.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.core.Context;
import org.firstinspires.ftc.teamcode.lib.Actions;
import org.firstinspires.ftc.teamcode.modules.Magazine;

@Autonomous(name = "Close", group = "A")
@Config
public class Close extends Auto {
    public static boolean runIntakeScore1 = true;
    public static boolean runGate = false;
    public static boolean runIntakeScore2 = true;
    public static boolean runIntakeGate = false;
    public static boolean runIntakeScore3 = true;

    public static int preloadTime = 0;
    public static int score1Time = 0;
    public static int score2Time = 0;

    public static int overridePathTime = 29250;

    private final Pose setupPose = ColorPose(112.3, 111.7, Math.toRadians(-2.8));
    private final Pose startPose = ColorPose(125.5, 120, Math.toRadians(36));

    private final Pose preloadScorePose = ColorPose(94, 94, Math.toRadians(-20));
    private final Pose scorePose = ColorPose(94, 87, Math.toRadians(36));
    private final Pose score2Control1 = ColorPose(124, 45, Math.toRadians(36));
    private final Pose scorePose3 = ColorPose(86, 102, Math.toRadians(36));
    private final Pose score3Control1 = ColorPose(122, 70, Math.toRadians(36));

    private final Pose intake1 = ColorPose(125, 84, Math.toRadians(0));
    private final Pose intake1Control1 = ColorPose(104, 83, 0);

    private final Pose gatePose = ColorPose(123, 73, Math.toRadians(0));
    private final Pose gateControl1 = ColorPose(112, 76, 0);

    private final Pose intake2Pose = ColorPose(132, 56, Math.toRadians(0));
    private final Pose intake2Control1 = ColorPose(90, 60, 0);

    private final Pose gateIntake = ColorPose(129.5, 63, Math.toRadians(20));
    private final Pose gateIntakeControl1 = ColorPose(92, 64, 0);
    private final Pose gateIntakeControl2 = ColorPose(117, 47, 0);

    private final Pose intake3Pose = ColorPose(127, 12, Math.toRadians(-80));
    private final Pose intake3Control1 = ColorPose(122, 70, 0);

    private final Pose parkInsidePose = ColorPose(86, 105, Math.toRadians(36));
    private final Pose parkOutsidePose = ColorPose(86, 120, Math.toRadians(36));

    @Override
    protected Pose getSetupPose() {
        return setupPose;
    }


    @Override
    protected void buildAutonomousSequence() {
        PathActionBuilder builder = new PathActionBuilder();

        builder.setStartPose(startPose);

        builder.action(robot.actions.prepareShooterClose())
                .setState(Magazine.IntakeState.FORWARD)
                .buildPath(path -> {
                    path.addLine(preloadScorePose);
                    path.setLinearHeading(
                            path.getStartPose().getHeading(), preloadScorePose.getHeading(), 0.5);
                })

                .await(() -> robot.opMode.getGameTimer().milliseconds() > preloadTime)
                .action(robot.actions.shootAll())
                .delay(delayMs)
                .run(() -> Context.motif = Context.allianceColor.equals(AllianceColor.RED)? robot.leftCamera.getObelisk() : robot.rightCamera.getObelisk());

        if (runIntakeScore1) {
            builder.actionAsync(robot.actions.intakeUntilFull())
                    .buildPath(path -> {
                        path.addCurve(intake1Control1, intake1);
                        path.setLinearHeading(
                                path.getStartPose().getHeading(), intake1.getHeading(), 0.1);
                    })
                    .delay(200)
            ;

//                    .await(() -> robot.magazine.getBallCount() == 3, 3000);
//                    .actionAsync(robot.actions.sortMagazineIfValid());
            if (runGate) {
                builder.buildPath(path -> {
                            path.addCurve(gateControl1, gatePose);
                            path.setLinearHeading(
                                    path.getStartPose().getHeading(), gatePose.getHeading(), 0.1);
                        })
                        .delay(1000);
            }
            builder.action(robot.actions.prepareShooterClose())
                    .buildPath(path -> {
                        path.addLine(scorePose);
                        path.setLinearHeading(
                                path.getStartPose().getHeading(), scorePose3.getHeading(), 0.8);
                    })
                    .await(() -> robot.opMode.getGameTimer().milliseconds() > score1Time)
                    .action(robot.actions.shootSorted());
        }

        if (runIntakeScore2) {
            builder.actionAsync(robot.actions.intakeUntilFull())
                    .buildPath(path -> {
                        path.addCurve(intake2Control1, intake2Pose);
                        path.setLinearHeading(
                                path.getStartPose().getHeading(), intake2Pose.getHeading(), 0.4);
                    })
//                    .await(() -> robot.magazine.getBallCount() == 3, 3000)
//                    .actionAsync(robot.actions.sortMagazineIfValid())

                    .action(robot.actions.prepareShooterClose())
                    .buildPath(path -> {
                        path.addCurve(score2Control1, scorePose);
                        path.setLinearHeading(
                                path.getStartPose().getHeading(), scorePose.getHeading(), 0.8);
                    })
                    .await(() -> robot.opMode.getGameTimer().milliseconds() > score2Time)
                    .action(robot.actions.shootSorted());
        }

        if (runIntakeGate) {
            builder.actionAsync(robot.actions.intakeUntilFull())
                    .buildPath(path -> {
                        path.addCurve(gateIntakeControl1, gateIntakeControl2, gateIntake);
                        path.setLinearHeading(
                                path.getStartPose().getHeading(), gateIntake.getHeading(), 0.4);
                    })
                    .delay(1200)
//                    .await(() -> robot.magazine.getBallCount() == 3, 3000)
//                    .actionAsync(robot.actions.sortMagazineIfValid())

                    .action(robot.actions.prepareShooterClose())
                    .buildPath(path -> {
                        path.addCurve(score2Control1, scorePose);
                        path.setLinearHeading(
                                path.getStartPose().getHeading(), scorePose.getHeading(), 0.8);
                    })
                    .action(robot.actions.shootSorted());
        }


        if (runIntakeScore3) {
            builder.actionAsync(robot.actions.intakeUntilFull())
                    .buildPath(path -> {
                        path.addCurve(intake3Control1, intake3Pose);
                        path.setLinearHeading(
                                path.getStartPose().getHeading(), intake3Pose.getHeading(), 0.4);
                    })

                    .delay(200)

//                    .await(() -> robot.magazine.getBallCount() == 3, 3000)
//                    .actionAsync(robot.actions.sortMagazineIfValid())

                    .action(robot.actions.prepareShooterClose())
                    .buildPath(path -> {
                        path.addCurve(intake3Control1, scorePose3);
                        path.setLinearHeading(
                                path.getStartPose().getHeading(), scorePose3.getHeading(), 0.8);
                    })
                    .action(robot.actions.shootSorted());
        }

        builder.setTimeOverride(overridePathTime, () -> {
            Pose currentPose = robot.follower.getPose();

            PathChain parkPath;

            if (Math.abs(currentPose.getY() - parkInsidePose.getY()) > Math.abs(currentPose.getY() - parkOutsidePose.getY())) {
                // Park outside
                Actions.cancelFor(robot.shooter);
                parkPath = robot.follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, parkInsidePose))
                        .setLinearHeadingInterpolation(currentPose.getHeading(), parkInsidePose.getHeading())
                        .build();
            } else {
                // Park inside
                parkPath = robot.follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, parkOutsidePose))
                        .setLinearHeadingInterpolation(currentPose.getHeading(), parkOutsidePose.getHeading())
                        .build();
            }

            robot.follower.followPath(parkPath, true);
        });

        robot.pathActionScheduler = builder.build();
    }
}
