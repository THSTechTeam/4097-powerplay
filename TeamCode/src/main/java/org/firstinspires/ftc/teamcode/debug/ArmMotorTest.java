package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.PIDController;

@TeleOp(name="Arm Motor Test", group="Test")
public class ArmMotorTest extends LinearOpMode {
    private DcMotorEx motorArm;

    private static final double kP = 0.025;
    private static final double kI = 1;
    private static final double kD = 0.0001;

    @Override
    public void runOpMode() throws InterruptedException {
        motorArm = hardwareMap.get(DcMotorEx.class, "motorArm");

        MotorConfigurationType motorConfigurationType = motorArm.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motorArm.setMotorType(motorConfigurationType);
        motorArm.setDirection(DcMotorSimple.Direction.REVERSE);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDController motorPID = new PIDController(kP, kI, kD, motorArm, DcMotorSimple.Direction.REVERSE);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Current Position", motorPID.getCurrentPosition());
            telemetry.update();
        }
    
        while (opModeIsActive()) {
            motorPID.setTargetPosition(360);
            motorPID.update();

            telemetry.addData("Current Position", motorPID.getCurrentPosition());
            telemetry.addData("Target Position", motorPID.getTargetPosition());
            telemetry.addData("Motor Power", motorPID.getMotorPower());
            telemetry.update();
        }
    }
}
