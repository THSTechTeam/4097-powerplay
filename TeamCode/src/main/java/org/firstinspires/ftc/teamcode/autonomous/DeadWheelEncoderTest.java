package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(name="Dead Wheel Encoder Test", group="Autonomous")
public class DeadWheelEncoderTest extends LinearOpMode {
    private Encoder rightEncoder;
    private Encoder leftEncoder;
    private Encoder centerEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motorFrontRight"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motorFrontLeft"));
        centerEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motorBackRight"));

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Right Encoder", rightEncoder.getCurrentPosition());
            telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
            telemetry.addData("Center Encoder", centerEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
