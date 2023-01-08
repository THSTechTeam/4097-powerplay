package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.opmode.DrivePowerConstants.highDrivePower;
import static org.firstinspires.ftc.teamcode.opmode.DrivePowerConstants.lowDrivePower;

@TeleOp(name="Mecanum Drive", group="TeleOp")
public class MecanumDrive extends LinearOpMode {
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorBackLeft;
    private DcMotorEx motorBackRight;
    private List<DcMotorEx> mecanumMotors;

    private final GamepadController gamepadController = new GamepadController();

    @Override
    public void runOpMode() throws InterruptedException {
        double motorPowerFactor = lowDrivePower;

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

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            gamepadController.update(gamepad1);

            double ly = -gamepadController.getStick(GamepadButton.LEFT_STICK_Y); // reversed
            double lx = gamepadController.getStick(GamepadButton.LEFT_STICK_X);
            double rx = gamepadController.getStick(GamepadButton.RIGHT_STICK_X);
            double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);

            motorPowerFactor = getDrivePowerFactor(motorPowerFactor);

            motorFrontLeft.setPower(((ly + lx + rx) / denominator) * motorPowerFactor);
            motorBackLeft.setPower(((ly - lx + rx) / denominator) * motorPowerFactor);
            motorFrontRight.setPower(((ly - lx - rx) / denominator) * motorPowerFactor);
            motorBackRight.setPower(((ly + lx - rx) / denominator) * motorPowerFactor);

            idle();
        }
    }

    private double getDrivePowerFactor(double previousPowerFactor) {
        if (!gamepadController.isPressed(GamepadButton.A)) {
            return previousPowerFactor;
        }

        if (previousPowerFactor == lowDrivePower) {
            return highDrivePower;
        } else {
            return lowDrivePower;
        }
    }
}
