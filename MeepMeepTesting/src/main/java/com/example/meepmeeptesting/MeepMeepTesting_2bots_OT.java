package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_2bots_OT {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        Pose2d beginPose = new Pose2d(-7.5, -62.85, Math.toRadians(-90));

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(beginPose)
                //.waitSeconds(0.5)
                .lineToYConstantHeading(-33)
//                .lineToYConstantHeading(-40)
//                .strafeToSplineHeading(new Vector2d(28, -43), Math.toRadians(60))



//            .splineToSplineHeading(new Pose2d(57,-43, Math.toRadians(120)),0)
//                .waitSeconds(2)                 //pick up sample 4
//
//            .turnTo(Math.toRadians(250))            //dropoff
//                .waitSeconds(1)
//
//            .turnTo(Math.toRadians(90))             //pick up sample 5
//                .waitSeconds(2)
//
//            .turnTo(Math.toRadians(250))            //dropoff
//                .waitSeconds(1)
//
//            .turnTo(Math.toRadians(60))             //pick up sample 6
//                .waitSeconds(2)
//
//            .turnTo(Math.toRadians(270))            //dropoff
//                .waitSeconds(1)

                .lineToY(-35)
                .splineToSplineHeading(new Pose2d(25,-43, Math.toRadians(270)),0)
                .splineToConstantHeading(new Vector2d(35,-20), Math.toRadians(270))



                .splineToConstantHeading(new Vector2d(43.2,-12),Math.toRadians(270))
//                .splineToSplineHeading(new Pose2d(43.5,-50, Math.toRadians(270)),Math.toRadians(270))

//                .splineToConstantHeading(new Vector2d(46,-11),Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(55,-16),Math.toRadians(270))
//                .splineToSplineHeading(new Pose2d(55,-50, Math.toRadians(270)),Math.toRadians(270))
//
//                .splineToConstantHeading(new Vector2d(58,-11),Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(58,-16),Math.toRadians(270))
                //.splineToSplineHeading(new Pose2d(58,-50, Math.toRadians(270)),Math.toRadians(270))
                //.splineToConstantHeading(new Vector2d(53,-50),Math.toRadians(270))




                .build());



        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(new Pose2d(30, 30, Math.toRadians(180)))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                //.addEntity(mySecondBot)
                .start();
    }
}
