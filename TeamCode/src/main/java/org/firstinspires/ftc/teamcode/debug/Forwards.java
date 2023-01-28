package org.firstinspires.ftc.teamcode.debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.teleop.MecanumDriveManager;

@Config
@TeleOp(name="Forwards", group="Debug")
public class Forwards extends LinearOpMode {
    public static double drivePower = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveManager driveManager = new MecanumDriveManager(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            driveManager.setMotorPowers(drivePower);

            telemetry.addData("Motor powers", driveManager.getMotorPowers());
            telemetry.update();
        }
    }
}
