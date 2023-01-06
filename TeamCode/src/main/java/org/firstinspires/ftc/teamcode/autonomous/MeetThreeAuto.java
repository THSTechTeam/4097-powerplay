package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PIDController;

import static org.firstinspires.ftc.teamcode.autonomous.ParkingLocationAnalyzer.ParkingLocation;

@Config
@Autonomous(name="Meet Three", group="Autonomous")
public class MeetThreeAuto extends LinearOpMode {
    public static double ONE_TILE_DISTANCE = 23; // in

    private ParkingLocationAnalyzer parkingLocationAnalyzer;
    private ParkingLocation parkingLocation;

    enum State {
        BACK_WALL_ADJUST, // Get off the back wall that you start on.
        TURN_1,           // Turn 90 degrees left.
        FORWARD_1,        // Move forward one tile.
        TURN_2,           // Turn 90 degrees right.
        FORWARD_2,        // Move forward two tiles.
        // The following states are only used if we need to go to the center or right parking positions.
        // Both the center and right parking locations start from the left parking location.
        TURN_3, // Turn 90 degrees right.
        FORWARD_CENTER,   // Move forward one tile.
        FORWARD_RIGHT,    // Move forward two tiles.
        RESET_HEADING,    // Reset the heading to 0 degrees.
        IDLE
    }

    private State currentState = State.IDLE;

    private Trajectory back_wall_adjust;
    private double turn_1;
    private Trajectory forward_1;
    private double turn_2;
    private Trajectory forward_2;
    private double turn_3;
    private Trajectory forward_center;
    private Trajectory forward_right;
    private double reset_heading_turn;

    // Arm setup.
    private PIDController armController;

    @Override
    public void runOpMode() throws InterruptedException {
        parkingLocationAnalyzer = new ParkingLocationAnalyzer(hardwareMap);

        // Roadrunner initialization.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        // Trajectories.
        back_wall_adjust = drive.trajectoryBuilder(new Pose2d())
                .forward(5)
                .build();

        turn_1 = Math.toRadians(90);

        Pose2d turn_1_end = new Pose2d(0, 0, turn_1);
        forward_1 = drive.trajectoryBuilder(back_wall_adjust.end().plus(turn_1_end))
                .forward(ONE_TILE_DISTANCE)
                .build();

        turn_2 = Math.toRadians(-90);

        Pose2d turn_2_end = new Pose2d(0, 0, turn_2);
        forward_2 = drive.trajectoryBuilder(forward_1.end().plus(turn_2_end))
                .forward(ONE_TILE_DISTANCE * 2)
                .build();

        // The following trajectories are only used if we need to go to the center or right parking positions.
        // Both the center and right parking locations start from the left parking location.
        turn_3 = Math.toRadians(-90);

        Pose2d turn_3_end = new Pose2d(0, 0, turn_3);
        forward_center = drive.trajectoryBuilder(forward_2.end().plus(turn_3_end))
                .forward(ONE_TILE_DISTANCE)
                .build();

        forward_right = drive.trajectoryBuilder(forward_2.end().plus(turn_3_end))
                .forward(ONE_TILE_DISTANCE * 2)
                .build();

        reset_heading_turn = Math.toRadians(90);

        // Arm PID controller.
        armController = new PIDController(
            0.005,
            0.0,
            0.2,
            hardwareMap.get(DcMotorEx.class, "motorArm"),
            DcMotorSimple.Direction.REVERSE
        );
        armController.setMaxMotorPower(1);

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
            parkingLocation = ParkingLocation.LEFT;
        }

        // Start the arm controller.
        armController.setTargetPosition(200);

        currentState = State.BACK_WALL_ADJUST;
        drive.followTrajectoryAsync(back_wall_adjust);

        while (opModeIsActive()) {
            switch (currentState) {
                case BACK_WALL_ADJUST:
                    if (!drive.isBusy()) {
                        currentState = State.TURN_1;
                        drive.turnAsync(Math.toRadians(90));
                    }
                    break;
                case TURN_1:
                    if (!drive.isBusy()) {
                        currentState = State.FORWARD_1;
                        drive.followTrajectoryAsync(forward_1);
                    }
                    break;
                case FORWARD_1:
                    if (!drive.isBusy()) {
                        currentState = State.TURN_2;
                        drive.turnAsync(Math.toRadians(-90));
                    }
                    break;
                case TURN_2:
                    if (!drive.isBusy()) {
                        currentState = State.FORWARD_2;
                        drive.followTrajectoryAsync(forward_2);
                    }
                    break;
                case FORWARD_2:
                    if (!drive.isBusy()) {
                        if (parkingLocation == ParkingLocation.LEFT) {
                            currentState = State.IDLE;
                        } else {
                            currentState = State.TURN_3;
                            drive.turnAsync(Math.toRadians(-90));
                        }
                    }
                    break;
                case TURN_3:
                    if (!drive.isBusy()) {
                        if (parkingLocation == ParkingLocation.CENTER) {
                            currentState = State.FORWARD_CENTER;
                            drive.followTrajectoryAsync(forward_center);
                        } else {
                            currentState = State.FORWARD_RIGHT;
                            drive.followTrajectoryAsync(forward_right);
                        }
                    }
                    break;
                case FORWARD_CENTER:
                case FORWARD_RIGHT:
                    if (!drive.isBusy()) {
                        currentState = State.RESET_HEADING;
                        drive.turnAsync(Math.toRadians(90));
                    }
                    break;
                case RESET_HEADING:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    // Set the arm down.
                    armController.setTargetPosition(50);
                    telemetry.addData("Idling", "...");
                    break;
            }

            drive.update();
            armController.update();
            telemetry.update();
        }
    }
}
