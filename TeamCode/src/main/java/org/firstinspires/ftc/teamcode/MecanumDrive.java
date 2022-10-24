package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Mecanum Drive", group="TeleOp")
public class MecanumDrive extends LinearOpMode {
    // General gamepad controller, all robot specific methods will be placed inside
    // the robot specific implementation of GamepadController.
    private class GamepadControllerBase {
        protected Gamepad gamepad;
        protected Gamepad previous;

        protected GamepadControllerBase() {
            gamepad  = new Gamepad();
            previous = new Gamepad();
        }

        // Must be called at the beginning of each while opModeIsActive() loop.
        // NOTE: 4097 driver station assignees controller to gamepad2 by default.
        protected void update() {
            try {
                previous.copy(gamepad);
                gamepad.copy(gamepad2);
            } catch (RobotCoreException e) {
                // Swallow exception, gamepad2 should always be valid.
            }
        }
    }

    private class GamepadController extends GamepadControllerBase {
        public static final double lowPowerFactor   = 0.3;
        private static final double highPowerFactor = 0.75;

        public double getPowerFactor(final double previousPowerFactor) {
            // Rising edge detector.
            if (!gamepadController.gamepad.a || gamepadController.previous.a) {
                return previousPowerFactor;
            }
    
            if (previousPowerFactor == lowPowerFactor) {
                return highPowerFactor;
            } else {
                return lowPowerFactor;
            }
        }
    }

    private final GamepadController gamepadController = new GamepadController();

    @Override
    public void runOpMode() throws InterruptedException {
        double motorPowerFactor = GamepadController.lowPowerFactor;

        DcMotor[] mecanumMotors = {
            hardwareMap.dcMotor.get("motorFrontLeft"),
            hardwareMap.dcMotor.get("motorBackLeft"),
            hardwareMap.dcMotor.get("motorFrontRight"),
            hardwareMap.dcMotor.get("motorBackRight"),
        };

        // Reverse left side drive motors.
        mecanumMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        mecanumMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            gamepadController.update();

            final double ly = -gamepadController.gamepad.left_stick_y; // reversed
            final double lx = -gamepadController.gamepad.left_stick_x; // reversed
            final double rx = gamepadController.gamepad.right_stick_x;
            final double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);

            final double[] motorPowers = {
                (ly + lx + rx) / denominator, // front left
                (ly - lx + rx) / denominator, // back left
                (ly - lx - rx) / denominator, // front right
                (ly + lx - rx) / denominator, // back right
            };

            motorPowerFactor = gamepadController.getPowerFactor(motorPowerFactor);

            for (int i = 0; i < mecanumMotors.length; i++) {
                mecanumMotors[i].setPower(motorPowers[i] * motorPowerFactor);
            }

            idle();
        }
    }
}
