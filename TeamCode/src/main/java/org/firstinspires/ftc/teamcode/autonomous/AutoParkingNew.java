package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.ParkingLocationAnalyzer;
import static org.firstinspires.ftc.teamcode.autonomous.ParkingLocationAnalyzer.ParkingLocation;

@Config
@Autonomous(name="Roadrunner Autonomous Parking", group="Autonomous")
public class AutoParkingNew extends LinearOpMode {
    // Regardless of parking position we first need to go to the center and push the signal cone out of the way.
    // If we are parking in the center we stop there, otherwise continue to the next point.
    public static Vector2d CENTER_PARKING_ONE = new Vector2d(0, 40);
    public static Vector2d CENTER_PARKING_TWO = new Vector2d(0, 30);

    // Left and right points continue from the center parking position.
    public static Vector2d LEFT_PARKING = new Vector2d(0, 0);
    public static Vector2d RIGHT_PARKING = new Vector2d(0, 0);

    private ParkingLocationAnalyzer parkingLocationAnalyzer;
    private ParkingLocation parkingLocation;

    private Trajectory forwards;
    private Trajectory left;
    private Trajectory right;

    @Override
    public void runOpMode() throws InterruptedException {
        parkingLocationAnalyzer = new ParkingLocationAnalyzer(hardwareMap);

        // Roadrunner initialization.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        // Trajectories.
        forwards = drive.trajectoryBuilder(new Pose2d())
            .splineToConstantHeading(CENTER_PARKING_ONE, 0)
            .splineToConstantHeading(CENTER_PARKING_TWO, 0)
            .build();

        left = drive.trajectoryBuilder(new Pose2d())
            .splineToConstantHeading(CENTER_PARKING_ONE, 0)
            .splineToConstantHeading(CENTER_PARKING_TWO, 0)
            .splineToConstantHeading(LEFT_PARKING, Math.toRadians(90))
            .build();

        right = drive.trajectoryBuilder(new Pose2d())
            .splineToConstantHeading(CENTER_PARKING_ONE, 0)
            .splineToConstantHeading(CENTER_PARKING_TWO, 0)
            .splineToConstantHeading(RIGHT_PARKING, Math.toRadians(-90))
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

        if (parkingLocation == ParkingLocation.LEFT) {
            drive.followTrajectory(left);
        } else if (parkingLocation == ParkingLocation.RIGHT) {
            drive.followTrajectory(right);
        } else {
            drive.followTrajectory(forwards);
        }
    }
}
