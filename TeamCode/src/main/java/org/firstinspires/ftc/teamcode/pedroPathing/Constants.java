package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    /**
     * Panels: 192.168.43.1:8001
     */
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(15.423) // acc. renisha, 34lb
//            .forwardZeroPowerAcceleration(-26.890)
//            .lateralZeroPowerAcceleration(-56.893)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.032, 0.035))
//            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.07, 0.02))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.001, 0.6, 0.01))
//            .centripetalScaling(0.002)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .xVelocity(77.993)
//            .yVelocity(58.094)
            .useBrakeModeInTeleOp(true);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2.25) // measured 11/29
            .strafePodX(-7.25) // measured 11/29
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
