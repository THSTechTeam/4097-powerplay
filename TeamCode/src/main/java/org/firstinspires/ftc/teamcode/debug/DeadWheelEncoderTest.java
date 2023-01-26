package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.roadrunner.StandardTrackingWheelLocalizer;

import java.util.Arrays;
import java.util.List;

@Autonomous(name="Dead Wheel Encoder Test", group="Debug")
public class DeadWheelEncoderTest extends LinearOpMode {
    private List<Double> wheelPositions;

    @Override
    public void runOpMode() throws InterruptedException {
        StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            wheelPositions = localizer.getWheelPositions();
            telemetry.addData("Left Encoder", wheelPositions.get(0));
            telemetry.addData("Right Encoder", wheelPositions.get(1));
            telemetry.addData("Front Encoder", wheelPositions.get(2));
            telemetry.update();
        }
    }
}
