package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.architecture.auto.Auto;
import org.firstinspires.ftc.teamcode.architecture.auto.pathaction.PathActionBuilder;

@Autonomous(name = "Straight and Back", group = "Test")
public class StraightAndBack extends Auto {
    private final Pose startPose = new Pose(10, 9, Math.toRadians(0));
    private final Pose cycleIntakePose = new Pose(55, 9, Math.toRadians(0));

    @Override
    protected Pose getSetupPose() {
        return startPose;
    }

    @Override
    protected void buildAutonomousSequence() {
        PathActionBuilder builder = new PathActionBuilder();

        builder.setStartPose(startPose);

        for (int i = 0; i < 100; i++) {
            builder.buildPath(path -> {
                       path.addLine(cycleIntakePose);
                       path.setConstantHeading(path.getStartPose().getHeading());
                   })
                    .buildPath(path -> {
                        path.addLine(startPose);
                        path.setConstantHeading(path.getStartPose().getHeading());
                    });
        }

        robot.pathActionScheduler = builder.build();
    }
}
