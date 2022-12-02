package org.firstinspires.ftc.teamcode.autonomous.showcase;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Strafe Left Right", group="showcase")
public class StrafeLeftRight extends LinearOpMode {
    private Trajectory forward;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        forward = drive.trajectoryBuilder(new Pose2d())
            .forward(30)
            .build();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // Call trajectories.
        drive.followTrajectory(forward);
    }
}