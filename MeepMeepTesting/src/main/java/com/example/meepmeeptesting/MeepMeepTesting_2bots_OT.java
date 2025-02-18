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

                .setReversed(true)
                .strafeTo(new Vector2d(35, -40), velFast, accMedium)
                .strafeTo(new Vector2d(45, -7), velFast, accMedium)     //goto at Sample4
                .setReversed(true)
                .strafeTo(new Vector2d(47, -40), velFast, accFast)    //pushSample4Home
                .strafeTo(new Vector2d(54, -8), velFast, accFast)     //toward Sample5
                .setReversed(true)
                .strafeTo(new Vector2d(57, -48), velFast, accMedium)    //pushSample5Home
                .strafeTo(new Vector2d(60, -8), velFast, accMedium)     //toward Sample6

                .setReversed(true)
//                .strafeToLinearHeading(new Vector2d(55, -10), Math.toRadians(-90),                    //start heading home with pushing sample6
//                        velFast, accMedium)
                .splineToLinearHeading(new Pose2d(60,-30, Math.toRadians(-90)),         //pushSample6Home, midway by Spline
                        Math.toRadians(-90), velFast, accMedium)
                .splineToLinearHeading(new Pose2d(40,-63.5, Math.toRadians(-90)),       //pushSample6Home, by Spline to Wall
                        Math.toRadians(-90), velFast, accMedium)


//                .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(-90),
//                        velFast, accMedium)  ////just before wall
//                .strafeToLinearHeading(new Vector2d(40, -63.5), Math.toRadians(-90),
//                        velFast, accMedium)  //to wall
                .waitSeconds(0.5)


                /** Score Spec1 */
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(3,-36),Math.toRadians(90))  //spline to submersible
                .splineToConstantHeading(new Vector2d(3,-35.5),Math.toRadians(90))  //straighten out robot with another spline to submersible
                .splineToConstantHeading(new Vector2d(3,-35),Math.toRadians(90))  //straighten out robot with another spline to submersible
                .waitSeconds(1)         //open claw at submersible



                /** Collect Spec2 */
                .strafeToLinearHeading(new Vector2d(3,-40),
                        Math.toRadians(-90), velFast, accMedium)                        //1st move back from submersible pole
                .splineToConstantHeading(new Vector2d(40,-62.5),Math.toRadians(-90))  //spline to more to near wall
                .splineToConstantHeading(new Vector2d(40,-63),Math.toRadians(-90))  //spline to more to near wall
                .splineToConstantHeading(new Vector2d(40,-63.5),Math.toRadians(-90))  //spline to more to near wall

                /** Score Spec2 */
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-1,-36),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-1,-35.5),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-1,-35),Math.toRadians(90))

                /** Collect Spec3 */
                .strafeToLinearHeading(new Vector2d(-1,-40),
                    Math.toRadians(-90), velFast, accMedium)  //1st move back from submersible pole
                .splineToConstantHeading(new Vector2d(40,-62.5),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40,-63),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40,-65),Math.toRadians(-90))

                /** Score Spec3 */
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-5,-36),Math.toRadians(90))   // previous x was -7
                .splineToConstantHeading(new Vector2d(-5,-35.5),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-5,-35),Math.toRadians(90))

                /** Collect Spec4 */
                .strafeToLinearHeading(new Vector2d(-5,-40),
                    Math.toRadians(-90), velFast, accMedium)  //1st move back from submersible pole
                .splineToConstantHeading(new Vector2d(40,-62.5),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40,-63),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40,-64.5),Math.toRadians(-90))

                /** Score Spec4 */
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-9,-36),Math.toRadians(90))   // previous x was -7
                .splineToConstantHeading(new Vector2d(-9,-35.5),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-9,-35),Math.toRadians(90))


                /** Park Home */
                .strafeToLinearHeading(new Vector2d(40,-64.5),
                        Math.toRadians(-90), velFast, accFast)  //1st move back from submersible pole

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