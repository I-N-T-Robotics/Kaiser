package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.58, -59.01, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(23.25, -33.22), Math.toRadians(11.31))
                                .splineTo(new Vector2d(46.59, -36.80), Math.toRadians(0.00))
                                .splineTo(new Vector2d(35.11, -53.18), Math.toRadians(224.34))
                                .splineTo(new Vector2d(4.61, -59.39), Math.toRadians(180.00))
                                .splineTo(new Vector2d(-27.34, -60.26), Math.toRadians(180.00))
                                .splineTo(new Vector2d(-56.38, -38.87), Math.toRadians(90.00))
                                .splineTo(new Vector2d(-44.93, -28.11), Math.toRadians(-4.22))
                                .splineTo(new Vector2d(-33.14, -53.96), Math.toRadians(-19.44))
                                .splineTo(new Vector2d(-13.19, -53.96), Math.toRadians(-4.76))
                                .splineTo(new Vector2d(49.27, -44.07), Math.toRadians(4.40))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}