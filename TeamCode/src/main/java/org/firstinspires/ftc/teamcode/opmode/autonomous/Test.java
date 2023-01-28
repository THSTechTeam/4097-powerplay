package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="Test Autonomous Parking", group="Autonomous")
public class Test extends LinearOpMode {
    public static double ONE_TILE_DISTANCE = 23; // in

    private TrajectorySequence forwards;
    private TrajectorySequence backwards;

    @Override
    public void runOpMode() throws InterruptedException {
        // Roadrunner initialization.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        forwards = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(30)
                .build();

        backwards = drive.trajectorySequenceBuilder(forwards.end())
                .back(30)
                .build();

        waitForStart();

        while (!isStopRequested()) {
            drive.followTrajectorySequence(forwards);
            drive.followTrajectorySequence(backwards);
        }
    }
}
