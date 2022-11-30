package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
import org.firstinspires.ftc.teamcode.autonomous.ParkingLocationAnalyzer;
import static org.firstinspires.ftc.teamcode.autonomous.ParkingLocationAnalyzer.ParkingLocation;

@Autonomous(name="Autonomous Parking", group="Autonomous")
public class AutoParking extends LinearOpMode {
    private static final double TILE_METER_LENGTH = 0.6;

    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorBackLeft;
    private DcMotorEx motorBackRight;
    private List<DcMotorEx> mecanumMotors;

    private BNO055IMU imu;

    private ParkingLocationAnalyzer parkingLocationAnalyzer;
    private ParkingLocation parkingLocation;

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
            driveForwardOneTile();
            resetHeading();
            strafeLeftOneTile();
            resetHeading();
        } else if (parkingLocation == ParkingLocation.CENTER || parkingLocation == null) {
            driveForwardOneTile();
            resetHeading();
        } else if (parkingLocation == ParkingLocation.RIGHT) {
            driveForwardOneTile();
            resetHeading();
            strafeRightOneTile();
            resetHeading();
        }
    }

    private void resetHeading() {
        int sleepBetweenStepTime = 250;
        double headingCorrectionPower = 0.05;
        double botHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        sleep(sleepBetweenStepTime);

        while (opModeIsActive() && botHeading != 0) {
            botHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            if (botHeading > 0) {
                motorFrontLeft.setPower(headingCorrectionPower);
                motorBackLeft.setPower(headingCorrectionPower);
                motorFrontRight.setPower(-headingCorrectionPower);
                motorBackRight.setPower(-headingCorrectionPower);
            } else if (botHeading < 0) {
                motorFrontLeft.setPower(-headingCorrectionPower);
                motorBackLeft.setPower(-headingCorrectionPower);
                motorFrontRight.setPower(headingCorrectionPower);
                motorBackRight.setPower(headingCorrectionPower);
            }
        }

        for (DcMotorEx motor : mecanumMotors) {
            motor.setPower(0);
        }

        sleep(sleepBetweenStepTime);
    }

    private void driveForwardOneTile() {
        // NOTE: Currently drives without the encoder.
        // Configured for Rev Robotics HD Hex Motor 20:1
        final double drivePower = 0.2;

        for (DcMotorEx motor : mecanumMotors) {
            motor.setPower(drivePower);
        }

        sleep((long)(TILE_METER_LENGTH / drivePower * 1100));

        for (DcMotorEx motor : mecanumMotors) {
            motor.setPower(0);
        }

        // Drive backwards slightly to clear the cone that we just pushed out of the way.
        for (DcMotorEx motor : mecanumMotors) {
            motor.setPower(-drivePower);
        }

        sleep((long)(TILE_METER_LENGTH / drivePower * 250));

        for (DcMotorEx motor : mecanumMotors) {
            motor.setPower(0);
        }
    }

    private void strafeLeftOneTile() {
        // NOTE: Currently drives without the encoder.
        // Configured for Rev Robotics HD Hex Motor 20:1
        final double drivePower = 0.15;

        motorFrontLeft.setPower(-drivePower);
        motorBackLeft.setPower(drivePower);
        motorFrontRight.setPower(drivePower);
        motorBackRight.setPower(-drivePower);

        sleep((long)(TILE_METER_LENGTH / drivePower * 1000));

        for (DcMotorEx motor : mecanumMotors) {
            motor.setPower(0);
        }
    }

    private void strafeRightOneTile() {
        // NOTE: Currently drives without the encoder.
        // Configured for Rev Robotics HD Hex Motor 20:1
        final double drivePower = 0.15;

        motorFrontLeft.setPower(drivePower);
        motorBackLeft.setPower(-drivePower);
        motorFrontRight.setPower(-drivePower);
        motorBackRight.setPower(drivePower);

        sleep((long)(TILE_METER_LENGTH / drivePower * 1000));

        for (DcMotorEx motor : mecanumMotors) {
            motor.setPower(0);
        }
    }
}
