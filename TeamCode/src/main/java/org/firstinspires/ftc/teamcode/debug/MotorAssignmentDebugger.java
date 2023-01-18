package org.firstinspires.ftc.teamcode.debug;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;

/*
 * This is a advanced opmode for debugging your motor configuration.
 * 
 * To use this opmode simply follow the instructions displayed in the telemetry.
 * 
 * The opmode will start by turning on one motor for 0.5 seconds, indicate which motor spun.
 * This will repeat for each motor.
 * 
 * At the end change your config based on the output of the opmode.
 * 
 * Button Mappings:
 *
 * Xbox/PS4 Button - Motor
 *   X / ▢         - Front Left
 *   Y / Δ         - Front Right
 *   B / O         - Rear  Right
 *   A / X         - Rear  Left
 *                                    The buttons are mapped to match the wheels spatially if you
 *                                    were to rotate the gamepad 45deg°. x/square is the front left
 *                    ________        and each button corresponds to the wheel as you go clockwise
 *                   / ______ \
 *     ------------.-'   _  '-..+              Front of Bot
 *              /   _  ( Y )  _  \                  ^
 *             |  ( X )  _  ( B ) |     Front Left   \    Front Right
 *        ___  '.      ( A )     /|       Wheel       \      Wheel
 *      .'    '.    '-._____.-'  .'       (x/▢)        \     (Y/Δ)
 *     |       |                 |                      \
 *      '.___.' '.               |          Rear Left    \   Rear Right
 *               '.             /             Wheel       \    Wheel
 *                \.          .'              (A/X)        \   (B/O)
 *                  \________/
 *
 * Uncomment the @Disabled tag below to use this opmode.
 */
@TeleOp(name="Motor Assignment Debugger", group="Debug")
public class MotorAssignmentDebugger extends LinearOpMode {
    private static final double MOTOR_POWER = 0.5;
    private static final long WAIT_TIME = 500;

    private enum Motor {
        FRONT_LEFT,
        FRONT_RIGHT,
        REAR_RIGHT,
        REAR_LEFT
    }

    private List<Pair<Motor, Motor>> motorSwapPairs = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addLine("Press play to begin the debugging opmode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        telemetry.addLine("Starting motor assignment debugging opmode");
        telemetry.addLine("The robot will turn on one motor for 0.5 seconds, indicate which motor spun");
        telemetry.addLine("This will repeat for each motor");
        telemetry.addLine("At the end change your config based on the output of the opmode");
        telemetry.addLine("Press x to continue");
        telemetry.update();

        while (!gamepad1.x) {
            idle();
        }

        telemetry.clearAll();
        telemetry.addLine("Spinning front left motor");
        telemetry.update();
        drive.setMotorPowers(MOTOR_POWER, 0, 0, 0);
        sleep(WAIT_TIME);
        drive.setMotorPowers(0, 0, 0, 0);

        telemetry.addLine("What motor spun?");
        outputButtonMotorMap();
        telemetry.update();

        waitForInput();

        if (gamepad1.x) {
            motorSwapPairs.add(new Pair<>(Motor.FRONT_LEFT, Motor.FRONT_LEFT));
        } else if (gamepad1.y) {
            motorSwapPairs.add(new Pair<>(Motor.FRONT_LEFT, Motor.FRONT_RIGHT));
        } else if (gamepad1.b) {
            motorSwapPairs.add(new Pair<>(Motor.FRONT_LEFT, Motor.REAR_RIGHT));
        } else if (gamepad1.a) {
            motorSwapPairs.add(new Pair<>(Motor.FRONT_LEFT, Motor.REAR_LEFT));
        }

        telemetry.clearAll();
        telemetry.addLine("Spinning front right motor");
        telemetry.update();
        drive.setMotorPowers(0, MOTOR_POWER, 0, 0);
        sleep(WAIT_TIME);
        drive.setMotorPowers(0, 0, 0, 0);

        telemetry.addLine("What motor spun?");
        outputButtonMotorMap();
        telemetry.update();

        waitForInput();

        if (gamepad1.x) {
            motorSwapPairs.add(new Pair<>(Motor.FRONT_RIGHT, Motor.FRONT_LEFT));
        } else if (gamepad1.y) {
            motorSwapPairs.add(new Pair<>(Motor.FRONT_RIGHT, Motor.FRONT_RIGHT));
        } else if (gamepad1.b) {
            motorSwapPairs.add(new Pair<>(Motor.FRONT_RIGHT, Motor.REAR_RIGHT));
        } else if (gamepad1.a) {
            motorSwapPairs.add(new Pair<>(Motor.FRONT_RIGHT, Motor.REAR_LEFT));
        }

        telemetry.clearAll();
        telemetry.addLine("Spinning rear right motor");
        telemetry.update();
        drive.setMotorPowers(0, 0, MOTOR_POWER, 0);
        sleep(WAIT_TIME);
        drive.setMotorPowers(0, 0, 0, 0);

        telemetry.addLine("What motor spun?");
        outputButtonMotorMap();
        telemetry.update();

        waitForInput();

        if (gamepad1.x) {
            motorSwapPairs.add(new Pair<>(Motor.REAR_RIGHT, Motor.FRONT_LEFT));
        } else if (gamepad1.y) {
            motorSwapPairs.add(new Pair<>(Motor.REAR_RIGHT, Motor.FRONT_RIGHT));
        } else if (gamepad1.b) {
            motorSwapPairs.add(new Pair<>(Motor.REAR_RIGHT, Motor.REAR_RIGHT));
        } else if (gamepad1.a) {
            motorSwapPairs.add(new Pair<>(Motor.REAR_RIGHT, Motor.REAR_LEFT));
        }

        telemetry.clearAll();
        telemetry.addLine("Spinning rear left motor");
        telemetry.update();
        drive.setMotorPowers(0, 0, 0, 1);
        sleep(WAIT_TIME);
        drive.setMotorPowers(0, 0, 0, 0);

        telemetry.addLine("What motor spun?");
        outputButtonMotorMap();
        telemetry.update();

        waitForInput();

        if (gamepad1.x) {
            motorSwapPairs.add(new Pair<>(Motor.REAR_LEFT, Motor.FRONT_LEFT));
        } else if (gamepad1.y) {
            motorSwapPairs.add(new Pair<>(Motor.REAR_LEFT, Motor.FRONT_RIGHT));
        } else if (gamepad1.b) {
            motorSwapPairs.add(new Pair<>(Motor.REAR_LEFT, Motor.REAR_RIGHT));
        } else if (gamepad1.a) {
            motorSwapPairs.add(new Pair<>(Motor.REAR_LEFT, Motor.REAR_LEFT));
        }

        telemetry.clearAll();
        telemetry.addLine("Finished debugging opmode");
        telemetry.addLine("Please change your config based on the output of the opmode");

        for (Pair<Motor, Motor> pair : motorSwapPairs) {
            telemetry.addLine(pair.first + " -> " + pair.second);
        }

        telemetry.update();

        // Pause the end of the opmode to allow for the user to change the current config.
        while (!opModeIsActive()) {
            idle();
        }
    }

    private void outputButtonMotorMap() {
        telemetry.addLine("X -> Front Left");
        telemetry.addLine("Y -> Front Right");
        telemetry.addLine("B -> Rear Right");
        telemetry.addLine("A -> Rear Left");
    }

    private void waitForInput() {
        while (!gamepad1.x && !gamepad1.y && !gamepad1.b && !gamepad1.a && !isStopRequested()) {
            idle();
        }
    }
}
