package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.modules.MagazineState.ArtifactColor.GREEN;
import static org.firstinspires.ftc.teamcode.modules.MagazineState.ArtifactColor.PURPLE;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.architecture.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.modules.MagazineState;

public final class Context {
    public static AllianceColor allianceColor = AllianceColor.BLUE;

    public static MagazineState motif = new MagazineState(GREEN, PURPLE, PURPLE);
    public static int shootCountBeforeOpening = 0;
    public static int shootCountAfterOpening = 0;
    public static boolean usedFrontStorage = false;

    public static boolean rightCameraStream = true, leftCameraStream = true;

    public static final Pose redApriltagPose = new Pose(72+58.27, 72+55.63, 54);
    public static final Pose blueApriltagPose = new Pose(72-58.27, 72+55.63, 54);

    public static final Pose redTargetPose = new Pose(144-10, 144-8);
    public static final Pose blueTargetPose = new Pose(10, 144-8);

    public static Pose pinpointPose;

    private Context() {}
}
