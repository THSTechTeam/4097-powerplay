package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Constants shared between opmodes.
 */
@Config
public class DriveConstants {
    public static final double TICKS_PER_REV = 537.6;
    public static final double MAX_RPM       = 312.5;

    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO   = 1;
    public static double TRACK_WIDTH  = 13.5; // TODO: Currently an estimate. Units in inches.

    // TODO: Currently tuned quite terribly but it was the best I could do without a perfectly flat drive surface.
    public static double kV      = 0.0175;
    public static double kA      = 0.002;
    public static double kStatic = 0.001;

    public static double MAX_VEL       = 25; // TODO: Further tuning for velocity required.
    public static double MAX_ACCEL     = 25; // TODO: Further tuning for acceleration required.
    public static double MAX_ANG_VEL   = Math.toRadians(100);
    public static double MAX_ANG_ACCEL = Math.toRadians(100);

    public static boolean FLIP_X        = false;
    public static boolean FLIP_Y        = false;
    public static boolean FLIP_ROTATION = false;

    public static double LOW_DRIVE_POWER  = 0.5;
    public static double HIGH_DRIVE_POWER = 0.75;

    public static double ARM_KP = 0.01;
    public static double ARM_KI = 0.0;
    public static double ARM_KD = 0.0002;
    public static double ARM_KF = 0.1;

    public static double ONE_TILE_DISTANCE = 23.0; // in

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
