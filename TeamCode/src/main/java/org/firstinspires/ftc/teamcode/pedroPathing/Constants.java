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
    public static FollowerConstants followerConstants =
            new FollowerConstants()
                    .mass(15.6)
                    .forwardZeroPowerAcceleration(-37.25559958109934)
                    .lateralZeroPowerAcceleration(-66.0228303965854)
                    .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.03, 0))
                    .headingPIDFCoefficients(new PIDFCoefficients(1.7, 0, 0.17, 0.01))
                    .drivePIDFCoefficients(
                            new FilteredPIDFCoefficients(0.08, 0, 0.003, 0.6, 0))
                    .centripetalScaling(0.0005);

    public static MecanumConstants driveConstants =
            new MecanumConstants()
                    .maxPower(1)
                    .rightFrontMotorName("fr")
                    .rightRearMotorName("br")
                    .leftRearMotorName("bl")
                    .leftFrontMotorName("fl")
                    .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .xVelocity(74.9825127068467)
                    .yVelocity(53.09580741341658)
                    .motorCachingThreshold(0.01)
                    .useVoltageCompensation(true)
                    .nominalVoltage(11.5);

    public static PinpointConstants localizerConstants =
            new PinpointConstants()
                    .forwardPodY(5.25) // Measured 1/29
                    .strafePodX(-7.375) // Measured 1/29
                    .distanceUnit(DistanceUnit.INCH)
                    .hardwareMapName("pinpoint")
                    .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
