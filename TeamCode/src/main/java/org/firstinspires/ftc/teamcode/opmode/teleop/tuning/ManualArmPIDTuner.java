package org.firstinspires.ftc.teamcode.opmode.tuning;

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
import static org.firstinspires.ftc.teamcode.DriveConstants.TICKS_PER_REV;

@Config
@TeleOp(name="Manual Arm PID Tuner", group="Tuning")
public class ManualArmPIDTuner extends LinearOpMode {
    public static double targetPosition = 0;

    private PIDFController armPIDController;

    @Override
    public void runOpMode() throws InterruptedException {
        armPIDController = new PIDFController(
            ARM_KP,
            ARM_KI,
            ARM_KD,
            ARM_KF,
            TICKS_PER_REV,
            hardwareMap.get(DcMotorEx.class, "motorArm"),
            DcMotorSimple.Direction.REVERSE
        );

        armPIDController.setMaxMotorPower(1);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("PID is busy", armPIDController.isBusy());
            telemetry.addData("Motor position", armPIDController.getCurrentPosition());
            telemetry.addData("Motor power", armPIDController.getPower());
            telemetry.addData("Motor target position", armPIDController.getTargetPosition());
            telemetry.update();
        }

        while (opModeIsActive()) {
            armPIDController.setTargetPosition(targetPosition);
            armPIDController.setConstants(ARM_KP, ARM_KI, ARM_KD, ARM_KF);

            telemetry.addData("PID is busy", armPIDController.isBusy());
            telemetry.addData("Motor position", armPIDController.getCurrentPosition());
            telemetry.addData("Motor power", armPIDController.getPower());
            telemetry.addData("Motor target position", armPIDController.getTargetPosition());
            telemetry.update();
        }
    }
}
