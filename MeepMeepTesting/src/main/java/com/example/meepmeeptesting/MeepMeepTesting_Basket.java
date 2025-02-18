package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_Basket {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 11)
                .setDimensions(14,14)
                .build();
        Pose2d beginPose = new Pose2d(-32, -62, Math.toRadians(0));     //TODO: would overide this for each case
        Pose2d beginPose_robot2 = new Pose2d(-16, -62, Math.toRadians(-90));


        Vector2d grabPosePosition = new Vector2d(-61, -48);
        Vector2d BasketDropPosition = new Vector2d(-55.5, -52.5);


        VelConstraint velSlow = new TranslationalVelConstraint(30);
        VelConstraint velMedium = new TranslationalVelConstraint(60);
        VelConstraint velFast = new TranslationalVelConstraint(70);
        AccelConstraint accSlow = new ProfileAccelConstraint(-30,30);
        AccelConstraint accMedium = new ProfileAccelConstraint(-60,60);
        AccelConstraint accFast = new ProfileAccelConstraint(-70,70);

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(beginPose)
                /** PreScore */
                .setTangent(45)
                .waitSeconds(0)
                .strafeToSplineHeading(new Vector2d(-57.5, -52.5), Math.toRadians(45))


        //TrajectoryActionBuilder grabPose = preScore.endTrajectory().fresh()
                .turnTo(Math.toRadians(80))

        //TrajectoryActionBuilder turnToBasket3 = grabPose.endTrajectory().fresh()
                .waitSeconds(0.6)
                .turnTo(Math.toRadians(45))

        // grabPose2 = turnToBasket3.endTrajectory().fresh()
                .turnTo(Math.toRadians(106))

        //TrajectoryActionBuilder turnToBasket20 = grabPose2.endTrajectory().fresh()
                .waitSeconds(0.5)
                .turnTo(Math.toRadians(45))

        //TrajectoryActionBuilder grabPose3 = turnToBasket3.endTrajectory().fresh()
                .turnTo(Math.toRadians(129))

        //TrajectoryActionBuilder turnToBasket10 = grabPose3.endTrajectory().fresh()
                .waitSeconds(0.5)
                .turnTo(Math.toRadians(45))

        //TrajectoryActionBuilder submersible = turnToBasket3.endTrajectory().fresh()
//                .setTangent(45)
//                .splineToSplineHeading(new Pose2d(-22.5, 0, 0), Math.toRadians(35))

        //TrajectoryActionBuilder turnToBasket2 = submersible.endTrajectory().fresh()
//                .setTangent(45)
//                .strafeToConstantHeading(new Vector2d(-25, 0))
//                .strafeToSplineHeading(new Vector2d(-59, -52.5), Math.toRadians(45))

        /** go to alliance sample */
                .waitSeconds(0.5)
                //.turnTo(Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-50, -52.5), Math.toRadians(-15))
                //.turnTo(Math.toRadians(-10))

        /** score alliance sample */
                .waitSeconds(0.6)
                .strafeToSplineHeading(new Vector2d(-57.5, -52.5),
                        Math.toRadians(45), velSlow, accSlow)
                //.turnTo(Math.toRadians(45))

        /** park at submersible */
                .waitSeconds(1)
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(-22.5, 0, 0),
                        Math.toRadians(35), velFast, accMedium)


                .build());






        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 11)
                .setDimensions(14,14)
                .build();


        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(beginPose_robot2)
                .strafeToConstantHeading(new Vector2d(15, -63),velSlow , accSlow)
                .strafeToConstantHeading(new Vector2d(-16, -63),velSlow , accSlow)
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                .addEntity(mySecondBot)
                .start();
    }
}