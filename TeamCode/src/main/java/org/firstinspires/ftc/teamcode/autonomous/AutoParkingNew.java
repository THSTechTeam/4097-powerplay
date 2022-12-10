package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.ParkingLocationAnalyzer;
import static org.firstinspires.ftc.teamcode.autonomous.ParkingLocationAnalyzer.ParkingLocation;

@Autonomous(name="Autonomous Parking", group="Autonomous")
public class AutoParking extends LinearOpMode {
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
        motorFrontLeft  = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackLeft   = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackRight  = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        mecanumMotors = Arrays.asList(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight);

        for (DcMotorEx motor : mecanumMotors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        parkingLocationAnalyzer = new ParkingLocationAnalyzer(hardwareMap);

        // Roadrunner initialization.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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
