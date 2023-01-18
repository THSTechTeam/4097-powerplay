package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.PIDFController;

import static org.firstinspires.ftc.teamcode.DriveConstants.ARM_KD;
import static org.firstinspires.ftc.teamcode.DriveConstants.ARM_KF;
import static org.firstinspires.ftc.teamcode.DriveConstants.ARM_KI;
import static org.firstinspires.ftc.teamcode.DriveConstants.ARM_KP;
import static org.firstinspires.ftc.teamcode.DriveConstants.HIGH_DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.DriveConstants.LOW_DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.DriveConstants.TICKS_PER_REV;

@Config
@TeleOp(name="Primary Drive", group="TeleOp")
public class PrimaryDrive extends LinearOpMode {
    private MecanumDriveManager drive;

    private PIDFController armPIDFController;

    public static int ARM_UP_POSITION      = 280;
    public static int ARM_MIDDLE_POSITION  = 200;
    public static int ARM_DOWN_POSITION    = 80;
    public static int ARM_REST_POSITION    = 20;
    public static int MANUAL_ARM_INCREMENT = 5;

    private final GamepadController gamepadController = new GamepadController();

    @Override
    public void runOpMode() throws InterruptedException {
        double motorPowerFactor = LOW_DRIVE_POWER;

        drive = new MecanumDriveManager(hardwareMap);

        armPIDFController = new PIDFController(
                ARM_KP,
                ARM_KI,
                ARM_KD,
                ARM_KF,
                TICKS_PER_REV,
                hardwareMap.get(DcMotorEx.class, "motorArm"),
                DcMotorSimple.Direction.REVERSE
            );

        armPIDFController.setMaxMotorPower(1);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("PID is busy", armPIDFController.isBusy());
            telemetry.addData("Motor position", armPIDFController.getCurrentPosition());
            telemetry.addData("Motor power", armPIDFController.getPower());
            telemetry.addData("Motor target position", armPIDFController.getTargetPosition());
            telemetry.update();
        }

        while (opModeIsActive()) {
            gamepadController.update(gamepad1);
            armPIDFController.update();

            if (gamepadController.isPressed(GamepadButton.DPAD_DOWN)) {
                armPIDFController.setTargetPosition(ARM_UP_POSITION);
            } else if (gamepadController.isPressed(GamepadButton.DPAD_RIGHT)) {
                armPIDFController.setTargetPosition(ARM_MIDDLE_POSITION);
            } else if (gamepadController.isPressed(GamepadButton.DPAD_LEFT)) {
                armPIDFController.setTargetPosition(ARM_DOWN_POSITION);
            } else if (gamepadController.isPressed(GamepadButton.DPAD_UP)) {
                armPIDFController.setTargetPosition(ARM_REST_POSITION);
            }

            if (gamepadController.isHeld(GamepadButton.RIGHT_TRIGGER)) {
                armPIDFController.setTargetPosition(
                        armPIDFController.getTargetPosition() + MANUAL_ARM_INCREMENT
                    );
            } else if (gamepadController.isHeld(GamepadButton.LEFT_TRIGGER)) {
                armPIDFController.setTargetPosition(
                        armPIDFController.getTargetPosition() - MANUAL_ARM_INCREMENT
                    );
            }

            if (gamepadController.isPressed(GamepadButton.RIGHT_BUMPER)) {
                armPIDFController.resetEncoder();
            }

            motorPowerFactor = getDrivePowerFactor(motorPowerFactor);

            drive.setWeightedDrivePower(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                motorPowerFactor
            );

            telemetry.addData("PID is busy", armPIDFController.isBusy());
            telemetry.addData("Motor position", armPIDFController.getCurrentPosition());
            telemetry.addData("Motor power", armPIDFController.getPower());
            telemetry.addData("Motor target position", armPIDFController.getTargetPosition());
            telemetry.update();
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
}
