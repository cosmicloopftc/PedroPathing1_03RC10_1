package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(7.5, -63.75, Math.toRadians(-90)))
                        .setTangent(-90)
                        .strafeToSplineHeading(new Vector2d(0, -36), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(27, -39), Math.toRadians(-90))
                        .setReversed(true)
                                .splineToLinearHeading(new Pose2d(27, -13, Math.toRadians(-90)), Math.toRadians(-90))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(35, -16, Math.toRadians(-90)), Math.toRadians(-90))
                //      .setReversed(true)

                      // .strafeToSplineHeading(new Vector2d(27, -39), Math.toRadians(-90))
//                        .strafeToSplineHeading(new Vector2d(27, -16), Math.toRadians(-90))
//                        .strafeToSplineHeading(new Vector2d(35, -16), Math.toRadians(-90))
                        .strafeToSplineHeading(new Vector2d(35, -57), Math.toRadians(-90))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(35, -13, Math.toRadians(-90)), Math.toRadians(-90))
                        .setReversed(false)
            //    .splineToLinearHeading(new Pose2d(35, -10, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(40, -16, Math.toRadians(-90)), Math.toRadians(-90))
                          //      .setReversed(true)
                .strafeToSplineHeading(new Vector2d(40, -57), Math.toRadians(-90))
//                        .strafeToSplineHeading(new Vector2d(35, -16), Math.toRadians(-90))
//                        .strafeToSplineHeading(new Vector2d(40, -16), Math.toRadians(-90))
//                        .strafeToSplineHeading(new Vector2d(40, -57), Math.toRadians(-90))
//                        .strafeToSplineHeading(new Vector2d(40, -16), Math.toRadians(-90))
//                        .strafeToSplineHeading(new Vector2d(45, -16), Math.toRadians(-90))
//                        .strafeToSplineHeading(new Vector2d(45, -57), Math.toRadians(-90))
        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
