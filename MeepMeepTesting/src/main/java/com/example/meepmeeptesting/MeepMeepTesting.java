package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1365, 1024);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(272.38911475873846), Math.toRadians(272.38911475873846), 12.75)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36,62 , 270))
                                .lineToLinearHeading(new Pose2d(36,40, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(43,39, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(33,44, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(70,30, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(78,30,Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(74,63,Math.toRadians(180)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}