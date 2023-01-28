package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.DriveConstants.ONE_TILE_DISTANCE;

@Config
@Autonomous(name="Delayed Parking", group="Autonomous")
public class DelayedParking extends LinearOpMode {
    public static double PARKING_DELAY = 25.0; // seconds
    private static final double SEC_TO_MS = 1000.0;

    private ParkingLocationAnalyzer parkingLocationAnalyzer;
    private ParkingLocation parkingLocation;

    private TrajectorySequence leftParking;
    private TrajectorySequence centerParking;
    private TrajectorySequence rightParking;

    @Override
    public void runOpMode() throws InterruptedException {
        parkingLocationAnalyzer = new ParkingLocationAnalyzer(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        leftParking = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(ONE_TILE_DISTANCE)
                .turn(Math.toRadians(90))
                .forward(ONE_TILE_DISTANCE)
                .turn(Math.toRadians(-90))
                .build();

        centerParking = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(ONE_TILE_DISTANCE)
                .turn(Math.toRadians(90))
                .forward(ONE_TILE_DISTANCE)
                .turn(Math.toRadians(-90))
                .build();

        rightParking = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(ONE_TILE_DISTANCE)
                .turn(Math.toRadians(90))
                .forward(ONE_TILE_DISTANCE)
                .turn(Math.toRadians(-90))
                .build();

        while (!isStarted() && !isStopRequested()) {
            ParkingLocation newParkingLocation = parkingLocationAnalyzer.getParkingLocation();

            if (newParkingLocation != null) {
                parkingLocation = newParkingLocation;
            }

            telemetry.addData("Parking Location", parkingLocation);
            telemetry.update();
        }

        if (parkingLocation == null) {
            parkingLocation = ParkingLocation.LEFT;
        }

        // Wait to park to allow for mare space for our alliance partner (if needed).
        sleep((long) (PARKING_DELAY * SEC_TO_MS));

        switch (parkingLocation) {
            case LEFT:
                drive.followTrajectorySequence(leftParking);
                break;
            case CENTER:
                drive.followTrajectorySequence(centerParking);
                break;
            case RIGHT:
                drive.followTrajectorySequence(rightParking);
                break;
        }
    }
}
