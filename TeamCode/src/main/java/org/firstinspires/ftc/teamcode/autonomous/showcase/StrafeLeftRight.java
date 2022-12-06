package org.firstinspires.ftc.teamcode.autonomous.showcase;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Strafe Left Right", group="showcase")
public class StrafeLeftRight extends LinearOpMode {
    private Trajectory trajectoryRight;
    private Trajectory trajectoryLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        trajectoryRight = drive.trajectoryBuilder(new Pose2d())
            .strafeRight(10)
            .build();

        trajectoryLeft = drive.trajectoryBuilder(new Pose2d())
            .strafeLeft(10)
            .build();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // Strafe left and right until stopped.
        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(trajectoryRight);
            sleep(1000);
            drive.followTrajectory(trajectoryLeft);
            sleep(1000);
        }
    }
}
