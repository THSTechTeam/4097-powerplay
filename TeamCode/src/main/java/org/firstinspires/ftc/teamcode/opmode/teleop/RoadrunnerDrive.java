package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.roadrunner.StandardTrackingWheelLocalizer;

import static org.firstinspires.ftc.teamcode.DriveConstants.HIGH_DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.DriveConstants.LOW_DRIVE_POWER;

@TeleOp(name="Roadrunner Field Centric Mecanum Drive", group="TeleOp")
public class RoadrunnerDrive extends LinearOpMode {
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorBackLeft;
    private DcMotorEx motorBackRight;
    private List<DcMotorEx> mecanumMotors;

    private final GamepadController gamepadController = new GamepadController();

    private StandardTrackingWheelLocalizer localizer;

    @Override
    public void runOpMode() throws InterruptedException {
        double driveMotorPowerFactor = LOW_DRIVE_POWER;

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

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        localizer.setPoseEstimate(new Pose2d());

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            gamepadController.update(gamepad1);
            localizer.update();

            // Used to reset the heading to account for drift.
            // WARNING: This can be dangerous to do accidentally BE CAREFUL!
            if (gamepadController.isPressed(GamepadButton.X)) {
                resetPoseEstimate();
            }

            double ly = -gamepadController.getStick(GamepadButton.LEFT_STICK_Y); // reversed
            double lx = gamepadController.getStick(GamepadButton.LEFT_STICK_X);
            double rx = gamepadController.getStick(GamepadButton.RIGHT_STICK_X);

            // Rotate the current powers by the inverse of the bot heading.
            double botHeading = -localizer.getPoseEstimate().getHeading();

            double adjustedLy  = ly * Math.cos(botHeading) + lx * Math.sin(botHeading);
            double adjustedLx  = -ly * Math.sin(botHeading) + lx * Math.cos(botHeading);
            double denominator = Math.max(Math.abs(adjustedLy) + Math.abs(adjustedLx) + Math.abs(rx), 1);

            driveMotorPowerFactor = getDrivePowerFactor(driveMotorPowerFactor);

            motorFrontLeft.setPower((adjustedLy + adjustedLx + rx) / denominator * driveMotorPowerFactor);
            motorBackLeft.setPower((adjustedLy - adjustedLx + rx) / denominator * driveMotorPowerFactor);
            motorFrontRight.setPower((adjustedLy - adjustedLx - rx) / denominator * driveMotorPowerFactor);
            motorBackRight.setPower((adjustedLy + adjustedLx - rx) / denominator * driveMotorPowerFactor);

            idle();
        }
    }

    private double getDrivePowerFactor(double previousPowerFactor) {
        if (!gamepadController.isPressed(GamepadButton.A)) {
            return previousPowerFactor;
        }

        if (previousPowerFactor == LOW_DRIVE_POWER) {
            return HIGH_DRIVE_POWER;
        } else {
            return LOW_DRIVE_POWER;
        }
    }

    private void resetPoseEstimate() {
        localizer.setPoseEstimate(new Pose2d());
    }
}
