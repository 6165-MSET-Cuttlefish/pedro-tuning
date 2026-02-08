package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.architecture.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.core.Context;

public class FieldPose {
    private FieldPose() {}

    public static Pose ColorPose(double x, double y, double heading) {
        if (Context.allianceColor.equals(AllianceColor.BLUE)){
            x = 144.0 - x;
            heading = Math.toRadians(180) - heading;
        }

        return new Pose(x, y, heading);
    }
}
