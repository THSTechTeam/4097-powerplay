package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.MecanumDriveManager;

@Config
@TeleOp(name="Meet Three", group="TeleOp")
public class MeetThree extends LinearOpMode {
    private MecanumDriveManager drive;

    private PIDController armPIDController;

    public static int ARM_UP_POSITION      = 280;
    public static int ARM_MIDDLE_POSITION  = 200;
    public static int ARM_DOWN_POSITION    = 80;
    public static int ARM_REST_POSITION    = 20;
    public static int MANUAL_ARM_INCREMENT = 5;

    private static class MotorPowerFactors {
        public static final double lowDrive  = 0.4;
        public static final double highDrive = 0.65;
    }

    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.2;

    private final GamepadController gamepadController = new GamepadController();

    @Override
    public void runOpMode() throws InterruptedException {
        double motorPowerFactor = MotorPowerFactors.lowDrive;

        drive = new MecanumDriveManager(hardwareMap);
        drive.flipY();

        armPIDController = new PIDController(
            kP, 
            kI, 
            kD,
            hardwareMap.get(DcMotorEx.class, "motorArm"),
            DcMotorSimple.Direction.REVERSE
        );
        armPIDController.setMaxMotorPower(1);

        Thread armPIDControllerThread = new Thread(armPIDController);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Motor position", armPIDController.getCurrentPosition());
            telemetry.addData("Motor power", armPIDController.getPower());
            telemetry.addData("Motor target position", armPIDController.getTargetPosition());
            telemetry.update();
        }

        armPIDControllerThread.start();

        while (opModeIsActive()) {
            gamepadController.update(gamepad1);

            if (gamepadController.isPressed(GamepadButton.DPAD_DOWN)) {
                armPIDController.setTargetPosition(ARM_UP_POSITION);
            } else if (gamepadController.isPressed(GamepadButton.DPAD_RIGHT)) {
                armPIDController.setTargetPosition(ARM_MIDDLE_POSITION);
            } else if (gamepadController.isPressed(GamepadButton.DPAD_LEFT)) {
                armPIDController.setTargetPosition(ARM_DOWN_POSITION);
            } else if (gamepadController.isPressed(GamepadButton.DPAD_UP)) {
                armPIDController.setTargetPosition(ARM_REST_POSITION);
            }

            if (gamepadController.isHeld(GamepadButton.RIGHT_TRIGGER)) {
                armPIDController.setTargetPosition(
                        armPIDController.getTargetPosition() + MANUAL_ARM_INCREMENT
                    );
            } else if (gamepadController.isHeld(GamepadButton.LEFT_TRIGGER)) {
                armPIDController.setTargetPosition(
                        armPIDController.getTargetPosition() - MANUAL_ARM_INCREMENT
                    );
            }

            if (gamepadController.isPressed(GamepadButton.RIGHT_BUMPER)) {
                armPIDController.resetEncoder();
            }

            motorPowerFactor = getDrivePowerFactor(motorPowerFactor);

            drive.setWeightedDrivePower(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                motorPowerFactor
            );

            telemetry.addData("PID is busy", armPIDController.isBusy());
            telemetry.addData("Motor position", armPIDController.getCurrentPosition());
            telemetry.addData("Motor power", armPIDController.getPower());
            telemetry.addData("Motor target position", armPIDController.getTargetPosition());
            telemetry.update();
            idle();
        }
    }

    private double getDrivePowerFactor(double previousPowerFactor) {
        if (!gamepadController.isPressed(GamepadButton.A)) {
            return previousPowerFactor;
        }

        if (previousPowerFactor == MotorPowerFactors.lowDrive) {
            return MotorPowerFactors.highDrive;
        } else {
            return MotorPowerFactors.lowDrive;
        }
    }
}
