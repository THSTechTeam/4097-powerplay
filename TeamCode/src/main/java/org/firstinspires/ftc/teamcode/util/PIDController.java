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
    private final double kP;
    private final double kI;
    private final double kD;
    private final DcMotor motor;

    private double targetPosition;
    private double currentError;
    private double lastError;
    
    private double timeStep;
    private double lastTime;
    private final ElapsedTime elapsedTime;

    private double maxMotorPower = 1.0;

    public PIDController(double kP, double kI, double kD, DcMotorEx motor, DcMotorSimple.Direction direction) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.elapsedTime = new ElapsedTime();

        this.motor = motor;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);
        
        // In all likelihood this behavior will never be used if the PID controller is used correctly.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 
        motor.setDirection(direction);
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;

        // Start the motor with proportional control.
        // This is done to simplify initialization of the lastTime variable.
        motor.setPower(kP * (targetPosition - motor.getCurrentPosition()));
        lastTime = elapsedTime.milliseconds();
    }

    public void update() {
        timeStep = elapsedTime.milliseconds() - lastTime;
        currentError = targetPosition - motor.getCurrentPosition();

        // Calculate the proportional, integral, and derivative terms.
        double p = kP * currentError;
        double i = kI * currentError * timeStep;
        double d = (kD * (currentError - lastError)) / timeStep;
        double power = p + i + d;

        // Normalize the power to be between -1 and 1.
        // Motor power input is limited to be between -1 and 1.
        power = Math.max(-maxMotorPower, Math.min(maxMotorPower, power));

        motor.setPower(power);
        lastError = currentError;
        lastTime = elapsedTime.milliseconds();
    }

    public void setMaxMotorPower(double maxMotorPower) {
        this.maxMotorPower = maxMotorPower;
    }

    public boolean isBusy() {
        return Math.abs(currentError) > 0.5;
    }

    public double getCurrentPosition() {
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
}
