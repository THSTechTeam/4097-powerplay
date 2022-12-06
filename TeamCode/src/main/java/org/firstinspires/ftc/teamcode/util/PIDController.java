package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private final double kP;
    private final double kI;
    private final double kD;
    private final DcMotor motor;

    private double targetPosition;
    private double error;
    private double errorStep;
    
    private double timeStep;
    private final ElapsedTime elapsedTime;

    public PIDController(double kP, double kI, double kD, DcMotor motor, DcMotorSimple.Direction direction) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.motor = motor;
        motor.setDirection(direction);
        this.elapsedTime = new ElapsedTime();
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;

        // Start the motor with just proportional control.
        motor.setPower(kP * (targetPosition - motor.getCurrentPosition()));
        timeStep = elapsedTime.milliseconds();
    }

    public void update() {
        timeStep = elapsedTime.milliseconds() - timeStep;
        error = targetPosition - motor.getCurrentPosition();
        double p = kP * error;
        double i = kI * error * timeStep;
        double d = (kD * (error - errorStep)) / timeStep;
        double power = p + i + d;

        power = Math.max(-1, Math.min(1, power));

        motor.setPower(power);
        errorStep = error;
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
}
