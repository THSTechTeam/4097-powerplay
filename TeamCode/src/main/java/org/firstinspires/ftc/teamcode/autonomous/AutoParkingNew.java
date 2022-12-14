package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.ParkingLocationAnalyzer;
import static org.firstinspires.ftc.teamcode.autonomous.ParkingLocationAnalyzer.ParkingLocation;

@Config
@Autonomous(name="Roadrunner Autonomous Parking", group="Autonomous")
public class AutoParkingNew extends LinearOpMode {
    public static double FORWARD_DISTANCE = 46; // in
    public static double STRAFE_DISTANCE  = 23; // in

    private ParkingLocationAnalyzer parkingLocationAnalyzer;
    private ParkingLocation parkingLocation;

    private Trajectory get_off_back_wall;

    private Trajectory left_parking_one;
    private Trajectory left_parking_two;

    @Override
    public void runOpMode() throws InterruptedException {
        parkingLocationAnalyzer = new ParkingLocationAnalyzer(hardwareMap);

        // Roadrunner initialization.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        // Trajectories.
        get_off_back_wall = drive.trajectoryBuilder(new Pose2d())
            .forward(5)
            .build();
        
        left_parking_one = drive.trajectoryBuilder(get_off_back_wall.end())
            .strafeLeft(STRAFE_DISTANCE)
            .build();

        left_parking_two = drive.trajectoryBuilder(left_parking_one.end())
            .forward(FORWARD_DISTANCE)
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

        if (parkingLocation == null) {
            parkingLocation = ParkingLocation.CENTER;
        }

        switch (parkingLocation) {
            case LEFT:
            default:
                drive.followTrajectory(get_off_back_wall);
                drive.followTrajectory(left_parking_one);
                drive.followTrajectory(left_parking_two);
        }
    }
}
