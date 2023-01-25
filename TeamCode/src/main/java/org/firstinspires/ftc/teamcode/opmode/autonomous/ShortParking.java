package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.DriveConstants.ONE_TILE_DISTANCE;

@Autonomous(name="Short Parking", group="Autonomous")
public class ShortParking extends LinearOpMode {
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
                .turn(Math.toRadians(-90))
                .forward(ONE_TILE_DISTANCE)
                .turn(Math.toRadians(90))
                .build();
        
        centerParking = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(ONE_TILE_DISTANCE)
                .turn(Math.toRadians(-90))
                .forward(ONE_TILE_DISTANCE)
                .turn(Math.toRadians(90))
                .build();

        rightParking = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(ONE_TILE_DISTANCE)
                .turn(Math.toRadians(-90))
                .forward(ONE_TILE_DISTANCE)
                .turn(Math.toRadians(90))
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
