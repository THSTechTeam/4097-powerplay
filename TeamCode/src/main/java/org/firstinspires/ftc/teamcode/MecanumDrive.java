package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Mecanum Drive", group="TeleOp")
public class MecanumDrive extends LinearOpMode {
    private DcMotor motorFrontLeft  = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft   = null;
    private DcMotor motorBackRight  = null;

    private final double lowPowerFactor  = 0.3;
    private final double highPowerFactor = 0.75;

    private double motorPowerFactor = lowPowerFactor;

    private double getPowerFactor(final double previousPowerFactor) {
        if (gamepad2.a) {
            return lowPowerFactor;
        } else if (gamepad2.b) {
            return highPowerFactor;
        } else {
            return previousPowerFactor;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // reverse left side motors
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            // 4097 driver station assignees controller to gamepad2 by default
            final double y = -gamepad2.left_stick_y; // reversed
            final double x = -(gamepad2.left_stick_x * 1.0); // imperfect strafing fix & reversed
            final double rx = gamepad2.right_stick_x;
            final double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double[] motorPowers = {
                (y + x + rx) / denominator, // front left
                (y - x + rx) / denominator, // back left
                (y - x - rx) / denominator, // front right
                (y + x - rx) / denominator, // back right
            };

            motorPowerFactor = getPowerFactor(motorPowerFactor);

            for (int i = 0; i < motorPowers.length; i++) {
                motorPowers[i] *= motorPowerFactor;
            }

            motorFrontLeft.setPower(motorPowers[0]);
            motorBackLeft.setPower(motorPowers[1]);
            motorFrontRight.setPower(motorPowers[2]);
            motorBackRight.setPower(motorPowers[3]);

            idle();
        }
    }
}
