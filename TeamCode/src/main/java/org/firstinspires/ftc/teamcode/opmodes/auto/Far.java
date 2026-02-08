package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.architecture.auto.FieldPose.ColorPose;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.auto.Auto;
import org.firstinspires.ftc.teamcode.architecture.auto.pathaction.PathActionBuilder;
import org.firstinspires.ftc.teamcode.lib.Actions;
import org.firstinspires.ftc.teamcode.modules.Turret;

@Autonomous(name = "Far", group = "A")
public class Far extends Auto {
    public static boolean runIntakeScore1 = true;
    public static boolean runIntakeScore2 = true;
    public static boolean runGate = true;

    public static int preloadTime = 0;
    public static int score1Time = 0;
    public static int score2Time = 0;
    public static int[] cycleTimes = {0, 0, 0, 0, 0};

    public static int overridePathTime = 29250;

    private final Pose setupPose = ColorPose(86, 14.7, Math.toRadians(90));
    private final Pose startPose = ColorPose(89, 16.2, Math.toRadians(73));
    private final Pose score1Pose = ColorPose(90, 14, Math.toRadians(45));
    private final Pose score1PoseControl1 = ColorPose(99.88372093023256, 53.3953488372093, Math.toRadians(45));

    private final Pose score2Pose = ColorPose(90, 13, Math.toRadians(30));

    private final Pose intake1Pose = ColorPose(129, 58, Math.toRadians(0));
    private final Pose intake1Control1 = ColorPose(95.55813953488372, 50.41860465116279, 0);
    private final Pose intake1Control2 = ColorPose(98.72093023255813, 60.837209302325576, 0);
    private final Pose intake1Control3 = ColorPose(107.46511627906978, 57.86046511627907, 0);


    private final Pose gateBackupPose = ColorPose(122, 59, Math.toRadians(0));
    private final Pose gatePose = ColorPose(125, 65.5, Math.toRadians(0));

    private final Pose intake2Pose = ColorPose(129, 35.5, Math.toRadians(0));
    private final Pose intake2Control1 = ColorPose(99.65116279069767, 28.46511627906977, 0);
    private final Pose intake2Control2 = ColorPose(101.51162790697674, 38.325581395348834, 0);

    private final Pose cycleIntakePose = ColorPose(135, 12, Math.toRadians(-7));
    private final Pose cycleScorePose = ColorPose(90, 11, Math.toRadians(0));

    private final Pose parkPose = ColorPose(107, 31, Math.toRadians(0));

    @Override
    protected Pose getSetupPose() {
        return setupPose;
    }

    @Override

    protected void buildAutonomousSequence() {
        PathActionBuilder builder = new PathActionBuilder();

        builder.setStartPose(startPose)
                .action(robot.actions.prepareShooterFar())
                .await(() -> robot.opMode.getGameTimer().milliseconds() > preloadTime)
                .action(robot.actions.shootAll())
                .delay(delayMs); //offset auto for alliance

        if (runIntakeScore1) {
            builder.actionAsync(robot.actions.intakeUntilFull())
                    .buildPath(path -> {

                        path.addCurve(intake1Control1, intake1Control2, intake1Control3, intake1Pose);
                        path.setLinearHeading(
                                path.getStartPose().getHeading(), intake1Pose.getHeading(), .1, .7);//.5);
                        path.setTimeoutConstraint(4);
                    });
//                    .await(() -> robot.magazine.getBallCount() == 3, 2000) BAD
//                    .run(() -> RobotLog.e("getBallCount1: " + robot.magazine.getBallCount()))
//                    .run(() -> RobotLog.e("Mag State in intakeScore1: " + robot.magazine.colorPattern));
//                    .actionAsync(robot.actions.sortMagazineIfValid());

            if (runGate) {
                builder.buildPath(path -> {
                           path.addLine(gateBackupPose);
                           path.setConstantHeading(gateBackupPose.getHeading());
                           path.addLine(gatePose);
                           path.setConstantHeading(gatePose.getHeading());
                            path.setTimeoutConstraint(2);
                       });
            }
            builder.buildPath(path -> {
                path.addCurve(score1PoseControl1, score1Pose);
//                        path.setLinearHeading(
//                                path.getStartPose().getHeading(), score1Pose.getHeading(), 0.8);
                path.setTangentHeading();
                path.setReversed();
            })

//            builder.action(robot.actions.prepareShooterFar())
//                    .buildPath(path -> {
//                        path.addCurve(score1PoseControl1, score1Pose);
////                        path.setLinearHeading(
////                                path.getStartPose().getHeading(), score1Pose.getHeading(), 0.8);
//                        path.setTangentHeading();
//                        path.setReversed();
//                    })
                    .await(() -> robot.opMode.getGameTimer().milliseconds() > score1Time)
                    .action(robot.actions.shootAll());
        }

        if (runIntakeScore2) {
            builder.actionAsync(robot.actions.intakeUntilFull())
                    .buildPath(path -> {
                        path.addCurve(intake2Control1, intake2Control2, intake2Pose);
                        path.setLinearHeading(
                                path.getStartPose().getHeading(), intake2Pose.getHeading(), 0.01, .6);//.4);
                        path.setTimeoutConstraint(4);
                    })
//                    .await(() -> robot.magazine.getBallCount() == 3, 2000) BAD
//                    .run(() -> RobotLog.e("getBallCount2: " + robot.magazine.getBallCount()))
//                    .run(() -> RobotLog.e("Mag State in intakeScore2: " + robot.magazine.colorPattern))
//                    .actionAsync(robot.actions.sortMagazineIfValid())

//                    .action(robot.actions.prepareShooterFar())
                    .buildPath(path -> {
                        path.addLine(score2Pose);
                        path.setLinearHeading(
                                path.getStartPose().getHeading(), score2Pose.getHeading(), 0.8);
                    })
                    .await(() -> robot.opMode.getGameTimer().milliseconds() > score2Time)
                    .action(robot.actions.shootAll());

        }

        for (int cycleNum = 0; cycleNum < 5; cycleNum++) {
            final int currentCycle = cycleNum;

            builder.actionAsync(robot.actions.intakeUntilFull())
                    .buildPath(path -> {
                        path.addLine(cycleIntakePose);
                        path.setLinearHeadingInterpolation(
                                path.getStartPose().getHeading(), cycleIntakePose.getHeading(), 0.01, 0.2)
                                .setTimeoutConstraint(3)
                                .setBrakingStrength(1);
                    })
//                    .await(() -> robot.magazine.getBallCount() == 3, 2000) BAd
//                    .action(robot.actions.prepareShooterFar())
                    .buildPath(path -> {
                        path.addLine(cycleScorePose);
                        path.setLinearHeading(
                                path.getStartPose().getHeading(), cycleScorePose.getHeading());
                    })
                    .await(()
                                    -> robot.opMode.getGameTimer().milliseconds()
                                    > cycleTimes[currentCycle])
                    .action(robot.actions.shootAll());
        }

        builder.setTimeOverride(overridePathTime, () -> {
            Actions.cancelFor(robot.shooter);
            Turret.TurretState.OFF.apply();

            Pose currentPose = robot.follower.getPose();
            PathChain parkPath = robot.follower.pathBuilder()
                                         .addPath(new BezierLine(currentPose, parkPose))
                                         .setLinearHeadingInterpolation(
                                                 currentPose.getHeading(), parkPose.getHeading())
                                         .build();

            robot.follower.followPath(parkPath, true);
        });

        robot.pathActionScheduler = builder.build();
    }

}
