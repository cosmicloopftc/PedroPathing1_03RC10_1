package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
                .build();
        Pose2d beginPose = new Pose2d(7.5, -63, Math.toRadians(-90));     //start at wall starting

        VelConstraint velSlow = new TranslationalVelConstraint(15);
        VelConstraint velMedium = new TranslationalVelConstraint(60);
        VelConstraint velFast = new TranslationalVelConstraint(90);
        AccelConstraint accSlow = new ProfileAccelConstraint(-15,15);
        AccelConstraint accMedium = new ProfileAccelConstraint(-60,60);
        AccelConstraint accFast = new ProfileAccelConstraint(-90,90);

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(beginPose)
                //wall to Submerge-score preload
                //move back and head to sample 3 area, get read to rotate push it home
                //from wall to Submerge, score 2nd specimen
                .setReversed(true)
                .lineToYConstantHeading(-37,velFast, accMedium)          //to submersible pole to preload score

                .setReversed(true)
                .lineToYConstantHeading(-40, velFast, accMedium)           //1st move back from submersible pole
                .strafeTo(new Vector2d(31, -40), velFast, accMedium)
                .splineToLinearHeading(new Pose2d(38,-20, Math.toRadians(-90)),         //goto at Sample4
                        Math.toRadians(90), velFast, accMedium)
                .splineToLinearHeading(new Pose2d(47,-14, Math.toRadians(-90)),         //goto at Sample4
                        Math.toRadians(-90), velFast, accMedium)

                .strafeTo(new Vector2d(44, -50), velFast, accMedium)  //pushSample4Home
                .strafeTo(new Vector2d(53, -8), velFast, accMedium)  //toward Sample5
                .strafeTo(new Vector2d(60, -48), velMedium, accMedium)   //pushSample5Home

                .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(-90),
                        velFast, accMedium)  ////just before wall
                .strafeToLinearHeading(new Vector2d(40, -63.5), Math.toRadians(-90),
                        velFast, accMedium)  //to wall
                .waitSeconds(0.5)


                /** already Spec1; now score Spec1 */
                .setReversed(true)
                .lineToYConstantHeading(-60)             //move away from wall
                .splineToConstantHeading(new Vector2d(3,-45),Math.toRadians(90))  //spline to submersible
                .splineToConstantHeading(new Vector2d(3,-40),Math.toRadians(90))  //straighten out robot with another spline to submersible
                .strafeToLinearHeading(new Vector2d(3,-35),
                        Math.toRadians(-90), velFast, accFast)          //accelerate to submersible pole to score
                //.lineToYConstantHeading(-35)                                      //accelerate to submersible pole to score
                .waitSeconds(1)         //open claw at submersible

                /** Collect Spec2 and score Spec2 */
                //.lineToYConstantHeading(-40)                                   //1st move back from submersible pole
                .strafeToLinearHeading(new Vector2d(3,-40),
                        Math.toRadians(-90), velMedium, accMedium)  //1st move back from submersible pole
                .splineToLinearHeading(new Pose2d(3,-45, Math.toRadians(-90)),         //goto at Sample4
                        Math.toRadians(-90), velFast, accMedium)
                //.splineTo(new Vector2d(3, -45), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40,-60),Math.toRadians(-90))  //spline to more to near wall
                .strafeTo(new Vector2d(40, -63.5), velFast, accMedium)  // to wall

                .build());



        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
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