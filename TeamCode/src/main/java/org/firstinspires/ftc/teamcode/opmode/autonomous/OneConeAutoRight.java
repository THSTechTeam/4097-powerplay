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

    public static double forwardOffset = 0;
    public static Vector2d scoreConeOne = new Vector2d(0, 0);
    public static Pose2d scoreConeTwo = new Pose2d(0, 0, 0);
    public static Pose2d resetPoseBeforeParking = new Pose2d(0, 0, 0);

    public static int waitForLiftTime = 3; // seconds
    public static int waitForScoreTime = 3; // seconds
    public static int distanceToCenter = 0; // in

    public static double armPower = 1.0;
    public static int armDrivePosition = 0;
    public static int armAbovePolePosition = 0;
    public static int armScorePosition = 0;

    private DcMotorEx armMotor;

    private TrajectorySequence scoreConeOnePath;
    private TrajectorySequence scoreConeTwoPath;
    private Trajectory resetBeforeParkingPath;

    private TrajectorySequence parkingPath;
    private TrajectorySequence parkLeftExtensionPath;
    private TrajectorySequence parkRightExtensionPath;

    @Override
    public void runOpMode() throws InterruptedException {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        parkingLocationAnalyzer = new ParkingLocationAnalyzer(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        scoreConeOnePath = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToConstantHeading(scoreConeOne, Math.toRadians(0))
                .addTemporalMarker(waitForLiftTime, () -> {
                    armMotor.setTargetPosition(armAbovePolePosition);
                })
                .addDisplacementMarker(() -> {
                    drive.followTrajectorySequence(scoreConeTwoPath);
                })
                .build();

        scoreConeTwoPath = drive.trajectorySequenceBuilder(scoreConeOnePath.end())
                .splineToSplineHeading(scoreConeTwo, Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    armMotor.setTargetPosition(armScorePosition);
                })
                .waitSeconds(waitForScoreTime)
                .build();
        
        resetBeforeParkingPath = drive.trajectoryBuilder(scoreConeTwoPath.end(), Math.toRadians(180)) // Drive backwards.
                .splineToSplineHeading(resetPoseBeforeParking, Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    armMotor.setTargetPosition(armDrivePosition);
                })
                .build();

        parkingPath = drive.trajectorySequenceBuilder(resetBeforeParkingPath.end())
                .forward(distanceToCenter)
                .build();

        parkLeftExtensionPath = drive.trajectorySequenceBuilder(parkingPath.end())
                .turn(Math.toRadians(90))
                .strafeLeft(ONE_TILE_DISTANCE)
                .turn(Math.toRadians(-90))
                .build();

        parkRightExtensionPath = drive.trajectorySequenceBuilder(parkingPath.end())
                .turn(Math.toRadians(-90))
                .strafeRight(ONE_TILE_DISTANCE)
                .turn(Math.toRadians(90))
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
        drive.followTrajectory(resetBeforeParkingPath);
        drive.followTrajectorySequence(parkingPath);

        if (parkingLocation == ParkingLocation.LEFT) {
            drive.followTrajectorySequence(parkLeftExtensionPath);
        } else if (parkingLocation == ParkingLocation.RIGHT) {
            drive.followTrajectorySequence(parkRightExtensionPath);
        }
    }
}
