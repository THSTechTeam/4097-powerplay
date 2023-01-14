package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Basic Motor PID Controller.
 * 
 * For more information on PID controllers, see https://docs.ftclib.org/ftclib/features/controllers.
 * 
 */
public class PIDController {
    protected double kP;
    protected double kI;
    protected double kD;
    protected final DcMotor motor;

    protected double targetPosition;
    protected double currentError;
    protected double lastError;
    protected boolean positionSet = false;

    protected double timeStep;
    protected double lastTime;
    protected final ElapsedTime elapsedTime;

    protected double maxMotorPower = 1.0;

    private boolean useEncoderConstraints = false;
    private double minEncoderConstraint;
    private double maxEncoderConstraint;

    public PIDController(
            double kP,
            double kI,
            double kD,
            DcMotorEx motor,
            DcMotorSimple.Direction direction
        ) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.elapsedTime = new ElapsedTime();

        this.motor = motor;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);
        
        // In all likelihood the break behavior will never be used if the PID controller is used correctly.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 
        motor.setDirection(direction);
    }

    public void setEncoderConstraints(double minEncoderConstraint, double maxEncoderConstraint) {
        this.minEncoderConstraint = minEncoderConstraint;
        this.maxEncoderConstraint = maxEncoderConstraint;
        useEncoderConstraints = true;
    }

    public void setTargetPosition(double targetPosition) {
        positionSet = true;

        if (useEncoderConstraints) {
            this.targetPosition = Math.max(minEncoderConstraint, Math.min(maxEncoderConstraint, targetPosition));
        } else {
            this.targetPosition = targetPosition;
        }

        // Start the motor with proportional control.
        // This is done to simplify initialization of the lastTime variable.
        motor.setPower(kP * (targetPosition - motor.getCurrentPosition()));
        lastTime = elapsedTime.milliseconds();
    }

    public void update() {
        if (!positionSet) {
            return;
        }

        timeStep = elapsedTime.milliseconds() - lastTime;
        currentError = targetPosition - motor.getCurrentPosition();

        // Calculate the proportional, integral, and derivative terms.
        double p = kP * currentError;
        double i = kI * currentError * timeStep;
        double d = (kD * (currentError - lastError)) / timeStep;
        double power = p + i + d;

        // Clip the power to the set maximum motor power.
        power = Math.max(-maxMotorPower, Math.min(maxMotorPower, power));

        motor.setPower(power);
        lastError = currentError;
        lastTime = elapsedTime.milliseconds();
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMaxMotorPower(double maxMotorPower) {
        this.maxMotorPower = maxMotorPower;
    }

    public void setConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public boolean isBusy() {
        return Math.abs(currentError) > 0.5;
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getMotorPower() {
        return motor.getPower();
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getTimeStep() {
        return timeStep;
    }

    public double getCurrentError() {
        return currentError;
    }

    public double getPower() {
        return motor.getPower();
    }
}
