package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.core.Context.blueTargetPose;
import static org.firstinspires.ftc.teamcode.core.Context.redTargetPose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.architecture.auto.pathaction.PathActionScheduler;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.architecture.hardware.Camera;
import org.firstinspires.ftc.teamcode.architecture.hardware.SRSHub;
import org.firstinspires.ftc.teamcode.architecture.hardware.SRSHubManager;
import org.firstinspires.ftc.teamcode.lib.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.lib.EnhancedTelemetry;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.Endgame;
import org.firstinspires.ftc.teamcode.modules.Magazine;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Robot {
    public static Robot robot;
    public static WriteToggles writeToggles = new WriteToggles();

    public static TelemetryToggles telemetryToggles =
            new TelemetryToggles();
    public static TelemetryToggles.ShooterTelemetry shooterTelemetry =
            new TelemetryToggles.ShooterTelemetry();
    public static TelemetryToggles.TurretTelemetry turretTelemetry =
            new TelemetryToggles.TurretTelemetry();
    public static TelemetryToggles.DrivetrainTelemetry drivetrainTelemetry =
            new TelemetryToggles.DrivetrainTelemetry();
    public static TelemetryToggles.EndgameTelemetry endgameTelemetry =
            new TelemetryToggles.EndgameTelemetry();
    public static TelemetryToggles.MagazineTelemetry magazineTelemetry =
            new TelemetryToggles.MagazineTelemetry();
    public static TelemetryToggles.AprilTagTelemetry aprilTagTelemetry =
            new TelemetryToggles.AprilTagTelemetry();
    public static TelemetryToggles.SRSHubTelemetry srsHubTelemetry =
            new TelemetryToggles.SRSHubTelemetry();

    public Drivetrain drivetrain;
    public Endgame endgame;
    public Shooter shooter;
    public Magazine magazine;
    public Turret turret;
    public Camera rightCamera, leftCamera;
    public SRSHubManager srsHubManager;

    public Follower follower;
    public PathActionScheduler pathActionScheduler;
    public final RobotActions actions;
    public Pose targetPose;
    public Pose cornerPose;

    public ElapsedTime endgameDecelTimer;

    public final EnhancedOpMode opMode;
    public final EnhancedTelemetry telemetry;

    public Robot(EnhancedOpMode opMode, boolean preservePosition)
            throws InterruptedException {
        this.opMode = opMode;
        this.telemetry = (EnhancedTelemetry) opMode.telemetry;

        Robot previousRobot = robot;
        setTargetPoseForAlliance();

        robot = this;

        pathActionScheduler = new PathActionScheduler();
        initializeSRSHub();
        initializeDrivetrain();
        initializeCamera();

        if (preservePosition && previousRobot != null && previousRobot.follower != null) {
            follower = Constants.createFollower(opMode.hardwareMap);
            follower.setPose(previousRobot.follower.getPose());
        } else {
            follower = Constants.createFollower(opMode.hardwareMap);
            follower.setPose(new Pose(72, 144 - 10, Math.PI/2));
        }

        initializeOtherModules();
        actions = new RobotActions(this);
    }

    public void updateWriteToggles() {
        boolean robotWriteEnabled = writeToggles.robotWrite;

        drivetrain.setWriteEnabled(robotWriteEnabled && writeToggles.drivetrainWrite);
        drivetrain.setTelemetryEnabled(drivetrainTelemetry.TOGGLE);

        endgame.setWriteEnabled(robotWriteEnabled && writeToggles.endgameWrite);
        endgame.setTelemetryEnabled(endgameTelemetry.TOGGLE);

        shooter.setWriteEnabled(robotWriteEnabled && writeToggles.shooterWrite);
        shooter.setTelemetryEnabled(shooterTelemetry.TOGGLE);

        magazine.setWriteEnabled(robotWriteEnabled && writeToggles.magazineWrite);
        magazine.setTelemetryEnabled(magazineTelemetry.TOGGLE);

        turret.setWriteEnabled(robotWriteEnabled && writeToggles.turretWrite);
        turret.setTelemetryEnabled(turretTelemetry.TOGGLE);

        srsHubManager.setWriteEnabled(robotWriteEnabled && writeToggles.srsHubWrite);
        srsHubManager.setTelemetryEnabled(srsHubTelemetry.TOGGLE);
    }

    private void initializeSRSHub() {
        srsHubManager = new SRSHubManager(opMode.hardwareMap);

        srsHubManager.addI2CDevice(1, new SRSHub.APDS9151());
        srsHubManager.addI2CDevice(2, new SRSHub.APDS9151());
        srsHubManager.addI2CDevice(3, new SRSHub.APDS9151());

        srsHubManager.initialize();
    }

    private void initializeDrivetrain() {
        drivetrain = new Drivetrain(opMode.hardwareMap);
        robot.endgameDecelTimer = new ElapsedTime();
    }

    private void initializeCamera() {
        int monitorId1 = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId1", "id", opMode.hardwareMap.appContext.getPackageName());
        int monitorId2 = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId2", "id", opMode.hardwareMap.appContext.getPackageName());

        rightCamera = new Camera(opMode.hardwareMap, "right", new Pose(7.5, -8.5, -90),
                new AprilTagProcessor.Builder()
                        .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                        .setLensIntrinsics(919.896, 919.896, 601.543, 309.905)
                        .build(),
                monitorId1, true);
        leftCamera = new Camera(opMode.hardwareMap, "left", new Pose(7.5, 8.5, 90),
                new AprilTagProcessor.Builder()
                        .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                        .setLensIntrinsics(919.896, 919.896, 601.543, 309.905)
                        .build(),
                monitorId2, true);
    }

    private void initializeOtherModules() {
        shooter = new Shooter(opMode.hardwareMap);
        turret = new Turret(opMode.hardwareMap);
        magazine = new Magazine(opMode.hardwareMap);
        endgame = new Endgame(opMode.hardwareMap);
    }

    private void setTargetPoseForAlliance() {
        switch (Context.allianceColor) {
            case RED:
                targetPose = redTargetPose;
                cornerPose = new Pose(144, 144);
                break;
            case BLUE:
                targetPose = blueTargetPose;
                cornerPose = new Pose(0, 144);
                break;
            default:
                break;
        }
    }

    public static class WriteToggles {
        public boolean shooterWrite = true;
        public boolean magazineWrite = true;
        public boolean turretWrite = true;
        public boolean drivetrainWrite = true;
        public boolean endgameWrite = true;
        public boolean srsHubWrite = true;
        public boolean robotWrite = true;
    }

    public static class TelemetryToggles {
        public boolean dsDebug = false;
        public boolean voltage = true;
        public boolean current = true;
        public static class ShooterTelemetry {
            public boolean TOGGLE = true;
            public boolean flywheel = true;
            public boolean hood = true;
            public boolean current = false;
        }

        public static class TurretTelemetry {
            public boolean TOGGLE = true;
            public boolean position = true;
            public boolean servos = false;
        }

        public static class DrivetrainTelemetry {
            public boolean TOGGLE = false;
            public boolean current = false;
        }

        public static class EndgameTelemetry {
            public boolean TOGGLE = true;
            public boolean current = false;
            public boolean initial = true;
            public boolean pto = false;
        }

        public static class MagazineTelemetry {
            public boolean TOGGLE = true;
            public boolean intake = false;
            public boolean vertical = false;
            public boolean servos = false;
            public boolean current = false;
            public boolean headlights = false;
            public boolean colorSensors = true;
        }

        public static class AprilTagTelemetry {
            public boolean TOGGLE = false;
            public boolean raw = false;
        }

        public static class SRSHubTelemetry {
            public boolean TOGGLE = false;
        }
    }
}