package com.example.meepmeeptesting;

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
                .setConstraints(45, 45, Math.toRadians(228.913), Math.toRadians(224.64475488950978), 5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(
                                new Pose2d(-32.08, 62.79, Math.toRadians(90)))
                                .lineToConstantHeading(new Vector2d(-35.08, 58))
                                .turn(Math.toRadians(-180))
                                .splineTo(new Vector2d(-35.08, 18), Math.toRadians(270))
                                .splineTo(new Vector2d(-35.08, 11), Math.toRadians(326))
                                .waitSeconds(1)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(-36, 13, Math.toRadians(180)))
                                .lineTo(new Vector2d(-9,15))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}