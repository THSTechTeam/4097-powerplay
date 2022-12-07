/*
 * FTC Team 4097 Meet Two teleop code
 * 
 * Now that we have a functional robot (and does more than just drive), we can start to have robot specific code
 * that won't just work on every robot with four wheels. Meet two is the first meet where we will hopefully be able to 
 * score a cone. This is the teleop program towards that gaol.
 */

package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.util.PIDController;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.opmode.GamepadInterface.GamepadController;
import static org.firstinspires.ftc.teamcode.opmode.GamepadInterface.GamepadButton;

@TeleOp(name="Meet Two", group="TeleOp")
public class MeetTwo extends LinearOpMode {
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorBackLeft;
    private DcMotorEx motorBackRight;
    private List<DcMotorEx> mecanumMotors;

    private DcMotorEx motorArm;
    private PIDController armPIDController;
    private ArmMotorPositions armMotorPosition;

    private static class MotorPowerFactors {
        public static final double lowDrive  = 0.3;
        public static final double highDrive = 0.6;
    }

    private static class ArmPIDConstants {
        public static final double kP = 0.025;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    private enum ArmMotorPositions {
        UP,     // Scoring position.
        MIDDLE, // In between scoring and grabbing position.
        DOWN,   // Grabbing position (Comes down on top of the cone).
        REST,   // Where the motor sits when not in use.
    }

    private final GamepadController gamepadController = new GamepadController();

    @Override
    public void runOpMode() throws InterruptedException {
        double motorPowerFactor = MotorPowerFactors.lowDrive;

        motorFrontLeft  = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackLeft   = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackRight  = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        mecanumMotors = Arrays.asList(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight);

        for (DcMotorEx motor : mecanumMotors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorArm = hardwareMap.get(DcMotorEx.class, "motorArm");

        armPIDController = new PIDController(
            ArmPIDConstants.kP, 
            ArmPIDConstants.kI, 
            ArmPIDConstants.kD,
            motorArm,
            DcMotorSimple.Direction.FORWARD
        );

        armPIDController.setMaxMotorPower(0.5);
        armMotorPosition = ArmMotorPositions.REST;
        setArmMotorPosition(armMotorPosition);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            gamepadController.update(gamepad1);
            armPIDController.update();

            double ly = -gamepadController.getLeftStickY(); // reversed
            double lx = gamepadController.getLeftStickX();
            double rx = gamepadController.getRightStickX();
            double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);

            motorPowerFactor = getDrivePowerFactor(motorPowerFactor);

            motorFrontLeft.setPower(((ly + lx + rx) / denominator) * motorPowerFactor);
            motorBackLeft.setPower(((ly - lx + rx) / denominator) * motorPowerFactor);
            motorFrontRight.setPower(((ly - lx - rx) / denominator) * motorPowerFactor);
            motorBackRight.setPower(((ly + lx - rx) / denominator) * motorPowerFactor);

            if (!armPIDController.isBusy()) {
                armMotorPosition = cycleArmMotorPosition();
                setArmMotorPosition(armMotorPosition);
            }

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

    private void setArmMotorPosition(ArmMotorPositions armMotorPosition) {
        // TODO: Needs to be tuned.
        switch (armMotorPosition) {
            case UP:
                armPIDController.setTargetPosition(0);
                break;
            case MIDDLE:
                armPIDController.setTargetPosition(0);
                break;
            case DOWN:
                armPIDController.setTargetPosition(0);
                break;
            case REST:
                armPIDController.setTargetPosition(0);
                break;
        }
    }

    private ArmMotorPositions cycleArmMotorPosition() {
        if (!gamepadController.isPressed(GamepadButton.B)) {
            return armMotorPosition;
        }

        // Follow the cycle pattern of UP -> MIDDLE -> DOWN -> UP -> ...
        // If the current position is REST, then the next position is MIDDLE.
        if (armMotorPosition == ArmMotorPositions.REST) {
            return ArmMotorPositions.MIDDLE;
        } else if (armMotorPosition == ArmMotorPositions.UP) {
            return ArmMotorPositions.MIDDLE;
        } else if (armMotorPosition == ArmMotorPositions.MIDDLE) {
            return ArmMotorPositions.DOWN;
        } else {
            return ArmMotorPositions.UP;
        }
    }
}
