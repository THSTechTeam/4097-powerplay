package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.ParkingLocationAnalyzer;
import static org.firstinspires.ftc.teamcode.autonomous.ParkingLocationAnalyzer.ParkingLocation;

@Autonomous(name="Roadrunner Autonomous Parking", group="Autonomous")
public class AutoParkingNew extends LinearOpMode {
    private static final double ONE_TILE = 23.622; // inches

    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorBackLeft;
    private DcMotorEx motorBackRight;
    private List<DcMotorEx> mecanumMotors;

    private ParkingLocationAnalyzer parkingLocationAnalyzer;
    private ParkingLocation parkingLocation;

    private Trajectory parkCenter;
    private Trajectory parkLeft;
    private Trajectory parkRight;

    @Override
    public void runOpMode() throws InterruptedException {
        parkingLocationAnalyzer = new ParkingLocationAnalyzer(hardwareMap);

        // Roadrunner initialization.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        parkCenter = drive.trajectoryBuilder(new Pose2d())
            .forward(ONE_TILE)
            .build();

        parkLeft = drive.trajectoryBuilder(new Pose2d())
            .forward(ONE_TILE)
            .strafeLeft(ONE_TILE)
            .build();
        
        parkRight = drive.trajectoryBuilder(new Pose2d())
            .forward(ONE_TILE)
            .strafeRight(ONE_TILE)
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
            drive.followTrajectory(parkLeft);
        } else if (parkingLocation == ParkingLocation.CENTER || parkingLocation == null) {
            drive.followTrajectory(parkCenter);
        } else if (parkingLocation == ParkingLocation.RIGHT) {
            drive.followTrajectory(parkRight);
        }
    }
}
