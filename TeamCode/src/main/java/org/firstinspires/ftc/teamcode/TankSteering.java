package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.net.CookieHandler;

@TeleOp(name="Tank Steering", group="TeleOp")
public class TankSteering extends LinearOpMode {
    private DcMotor motorRight = null;
    private DcMotor motorLeft = null;

    @Override
    public void runOpMode() throws InterruptedException {
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            motorRight.setPower(-gamepad1.left_stick_y);
            motorLeft.setPower(-gamepad1.right_stick_y);

            idle();
        }
    }
}
