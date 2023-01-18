package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;

@Autonomous(name="Roadrunner Async", group="Autonomous")
public class RoadrunnerAsync extends OpMode {
    private SampleMecanumDrive drive;

    private Trajectory forward;
    private Trajectory backward;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        forward = drive.trajectoryBuilder(new Pose2d())
                .forward(10)
                .addDisplacementMarker(() -> {
                    drive.followTrajectoryAsync(backward);
                })
                .build();

        backward = drive.trajectoryBuilder(forward.end())
                .back(10)
                .addDisplacementMarker(() -> {
                    drive.followTrajectoryAsync(forward);
                })
                .build();

        drive.followTrajectoryAsync(forward);
    }

    @Override
    public void loop() {
        drive.update();
    }
}
