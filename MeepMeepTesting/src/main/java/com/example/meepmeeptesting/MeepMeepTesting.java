package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    static final double TILE_METER_SIZE = 0.6;
    static final double METERS_TO_INCHES = 39.3701;

    public static Vector2d CENTER_PARKING_ONE = new Vector2d(0, 40);
    public static Vector2d CENTER_PARKING_TWO = new Vector2d(0, 30);

    // Left and right points continue from the center parking position.
    public static Vector2d LEFT_PARKING = new Vector2d(-30, 30);
    public static Vector2d RIGHT_PARKING = new Vector2d(0, 0);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                .strafeRight(22.62)
//                                .splineToSplineHeading(new Pose2d(38, -34, Math.toRadians(210)), Math.toRadians(0))
                                .forward(15)
                                .turn(Math.toRadians(-130))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
