package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class PIDFController extends PIDController {
    private final double kF;
    private final double ticksPerDegree;

    public PIDFController(
            double kP,
            double kI,
            double kD,
            double kF,
            double ticksPerRotation,
            DcMotorEx motor, 
            DcMotorSimple.Direction direction
        ) {
        super(kP, kI, kD, motor, direction);
        this.kF = kF;
        this.ticksPerDegree = ticksPerRotation / 360;
    }

    @Override
    public void update() {
        if (!positionSet) {
            return;
        }

        timeStep = elapsedTime.seconds() - lastTime;
        currentError = targetPosition - motor.getCurrentPosition();

        double p = kP * currentError;
        double i = kI * currentError * timeStep;
        double d = kD * (currentError - lastError) / timeStep;
        double ff = kF * targetPosition / ticksPerDegree;
        double power = p + i + d + ff;

        // Clip the power to the set maximum motor power.
        power = Math.max(-maxMotorPower, Math.min(maxMotorPower, power));

        motor.setPower(power);
        lastError = currentError;
        lastTime = elapsedTime.seconds();
    }

    @Override
    public void setConstants(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
}
