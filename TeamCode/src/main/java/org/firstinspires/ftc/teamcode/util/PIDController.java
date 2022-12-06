package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        // Start the motor with proportional control.
        // This is done to simplify initialization of the timeStep variable.
        motor.setPower(kP * (targetPosition - motor.getCurrentPosition()));
        timeStep = elapsedTime.milliseconds();
    }

    public void update() {
        timeStep = elapsedTime.milliseconds() - timeStep;
        error = targetPosition - motor.getCurrentPosition();

        // Calculate the proportional, integral, and derivative terms.
        double p = kP * currentError;
        double i = kI * currentError * timeStep;
        double d = (kD * (error - lastError)) / timeStep;
        double power = p + i + d;

        // Normalize the power to be between -1 and 1.
        // Motor power input is limited to be between -1 and 1.
        power = Math.max(-1, Math.min(1, power));

        motor.setPower(power);
        lastError = currentError;
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
