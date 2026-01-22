package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.HeadingPath;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(18,16)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49.9, -49.7, Math.toRadians(230)))
                .strafeToLinearHeading(new Vector2d(-29.1,-29),Math.toRadians(230),new TranslationalVelConstraint(13))
                .waitSeconds(1)
                .setTangent(Math.toRadians(70))
                .splineToLinearHeading(new Pose2d(-23.3,-24,Math.toRadians(250)), Math.toRadians(15),
                        new TranslationalVelConstraint(40))

                .splineToSplineHeading(new Pose2d(-12,-55,Math.toRadians(270)),Math.toRadians(270),
                        new TranslationalVelConstraint(40))
                //.splineToLinearHeading(new Pose2d(-14.5,-49.3,Math.toRadians(270)), Math.toRadians(200))
                .waitSeconds(0.1)
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(-29,-22,Math.toRadians(230)),Math.toRadians(150))
//                .splineTo(new Vector2d(-21.9,-22),Math.toRadians(130))
                .waitSeconds(3)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(6,-24.8,Math.toRadians(270)), Math.toRadians(310),
                        new TranslationalVelConstraint(40))
                .splineToSplineHeading(new Pose2d(11.5,-56.5,Math.toRadians(270)),Math.toRadians(270)

                        ,new TranslationalVelConstraint(40))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(-28.5,-22.7), Math.toRadians(230))
                .waitSeconds(3.5)
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(26.5 ,-24,Math.toRadians(270)), Math.toRadians(330),
                        new TranslationalVelConstraint(40)
                )
                .splineToLinearHeading(new Pose2d(36, -60,Math.toRadians(270)), Math.toRadians(270),
                        new TranslationalVelConstraint(40))
                .strafeToLinearHeading(new Vector2d(-44,-23), Math.toRadians(246))
                .waitSeconds(3.5)
                .build());

        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(18,16)
                .build();
        bot2.runAction(bot2.getDrive().actionBuilder(new Pose2d(-49.5, 49.5, Math.toRadians(130)))
                .strafeToLinearHeading(new Vector2d(-29.1,29),Math.toRadians(360-230),new TranslationalVelConstraint(13))
                .waitSeconds(1)
                .setTangent(Math.toRadians(360-70))
                .splineToLinearHeading(new Pose2d(-23.3,24,Math.toRadians(360-250)), Math.toRadians(360-15),
                        new TranslationalVelConstraint(40))
                .splineToSplineHeading(new Pose2d(-12,55,Math.toRadians(360-270)),Math.toRadians(360-270),
                        new TranslationalVelConstraint(40))
                //.splineToLinearHeading(new Pose2d(-14.5,-49.3,Math.toRadians(270)), Math.toRadians(200))
                .waitSeconds(0.1)
                .setTangent(Math.toRadians(360-45))
                .splineToSplineHeading(new Pose2d(-29,22,Math.toRadians(360-230)),Math.toRadians(360-150))
//                .splineTo(new Vector2d(-21.9,-22),Math.toRadians(130))
                .waitSeconds(3)
                .setTangent(Math.toRadians(360-45))
                .splineToLinearHeading(new Pose2d(6,24.8,Math.toRadians(360-270)), Math.toRadians(360-310),
                        new TranslationalVelConstraint(40))
                .splineToSplineHeading(new Pose2d(11.5,56.5,Math.toRadians(360-270)),Math.toRadians(360-270)

                        ,new TranslationalVelConstraint(40))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(-28.5,22.7), Math.toRadians(360-230))
                .waitSeconds(3.5)
                .setTangent(Math.toRadians(360-45))
                .splineToSplineHeading(new Pose2d(26.5 ,24,Math.toRadians(360-270)), Math.toRadians(360-330),
                        new TranslationalVelConstraint(40)
                )
                .splineToLinearHeading(new Pose2d(36, 58,Math.toRadians(360-270)), Math.toRadians(360-270),
                        new TranslationalVelConstraint(40))
                .strafeToLinearHeading(new Vector2d(-40.4,21), Math.toRadians(360-242))
                .waitSeconds(3.5)
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(bot2)
                .start();

    }
}