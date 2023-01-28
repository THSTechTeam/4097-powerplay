package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.DriveConstants.ONE_TILE_DISTANCE;

@Config
@Autonomous(name="One Cone Right + Park", group="Autonomous")
public class OneConeAutoRight extends LinearOpMode {
    private ParkingLocationAnalyzer parkingLocationAnalyzer;
    private ParkingLocation parkingLocation;

    public static int waitForStartTime = 2; // seconds
    public static int waitForLiftTime = 1; // seconds
    public static int waitForScoreTime = 3; // seconds

    public static double armPower = 0.3;
    public static int armDrivePosition = 10;
    public static int armAbovePolePosition = 110;
    public static int armScorePosition = 65;

    public static double x = 9.2;
    public static double y = 3.2;

    private DcMotorEx armMotor;

    private TrajectorySequence scoreConeOnePath;
     private TrajectorySequence scoreConeTwoPath;
     private Trajectory resetBeforeParkingPath;

     private TrajectorySequence parkingCenterPath;
     private TrajectorySequence parkLeftPath;
     private TrajectorySequence parkRightPath;

    @Override
    public void runOpMode() throws InterruptedException {
        armMotor = hardwareMap.get(DcMotorEx.class, "motorArm");

        parkingLocationAnalyzer = new ParkingLocationAnalyzer(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        scoreConeOnePath = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(waitForStartTime)
                .splineToConstantHeading(new Vector2d(3, 0), Math.toRadians(0))
                .addTemporalMarker(waitForLiftTime, () -> {
                    armMotor.setTargetPosition(armAbovePolePosition);
                })
                .build();

        scoreConeTwoPath = drive.trajectorySequenceBuilder(scoreConeOnePath.end())
                .splineToSplineHeading(new Pose2d(x, y, Math.toRadians(30)), Math.toRadians(90))
                .addTemporalMarker(waitForScoreTime, () -> {
                    armMotor.setTargetPosition(armScorePosition);
                })
                .waitSeconds(waitForScoreTime)
                .UNSTABLE_addTemporalMarkerOffset((0.00001), () -> { // Start immediately.
                    armMotor.setTargetPosition(armAbovePolePosition);
                })
                .waitSeconds(waitForLiftTime)
                .build();

        resetBeforeParkingPath = drive.trajectoryBuilder(scoreConeTwoPath.end(), Math.toRadians(180)) // Drive backwards.
                .splineToSplineHeading(new Pose2d(0, 1, Math.toRadians(0)), Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    armMotor.setTargetPosition(armDrivePosition);
                })
                .build();

        parkingCenterPath = drive.trajectorySequenceBuilder(resetBeforeParkingPath.end())
                .lineToConstantHeading(new Vector2d(28, 0))
                .build();

        parkLeftPath = drive.trajectorySequenceBuilder(resetBeforeParkingPath.end())
                .lineToConstantHeading(new Vector2d(35, 0))
                .lineToConstantHeading(new Vector2d(20, 0))
                .waitSeconds(0.2)
                .splineToSplineHeading(new Pose2d(28, 27, Math.toRadians(90)), Math.toRadians(90))
                .build();

        parkRightPath = drive.trajectorySequenceBuilder(resetBeforeParkingPath.end())
                .lineToConstantHeading(new Vector2d(35, 0))
                .lineToConstantHeading(new Vector2d(20, 0))
                .waitSeconds(0.2)
                .splineToSplineHeading(new Pose2d(28, -23, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        // vvv The following loop replaces `waitForStart()`. vvv
        while (!isStarted() && !isStopRequested()) {
            ParkingLocation newParkingLocation = parkingLocationAnalyzer.getParkingLocation();

            if (newParkingLocation != null) {
                parkingLocation = newParkingLocation;
            }

            telemetry.addData("Parking Location", parkingLocation);
            telemetry.update();
        }
        // ^^^ End of `waitForStart()` replacement. ^^^

        armMotor.setTargetPosition(armDrivePosition);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armMotor.setPower(armPower);

        drive.followTrajectorySequence(scoreConeOnePath);
        drive.followTrajectorySequence(scoreConeTwoPath);
        drive.followTrajectory(resetBeforeParkingPath);

        
        if (parkingLocation == ParkingLocation.LEFT) {
            drive.followTrajectorySequence(parkLeftPath);
        } else if (parkingLocation == ParkingLocation.RIGHT) {
            drive.followTrajectorySequence(parkRightPath);
        } else {
            drive.followTrajectorySequence(parkingCenterPath);
        }
    }
}
