package OLD;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RR.MecanumDrive;

//@Config
//@Autonomous(name = "RR Auto Red Specimen v1.1", group = "Auto")

public final class AutoRedSpecimenRR extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(7.5, -63.5, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(7.5, -63.5, Math.toRadians(-90)))
                //    .lineToY(-33)
//                    .splineToConstantHeading(new Vector2d(0, -32), Math.toRadians(90))// score preload
//
//                    .lineToY(-45)
//                        .setTangent(Math.toRadians(0))
//                    .lineToX(20)
//                        .setTangent(Math.toRadians(90))
//                    .lineToY(-16)
//                        .setTangent(Math.toRadians(0))
//                    .lineToX(30)
//                        .setTangent(Math.toRadians(90))
//                    .lineToY(-57)  // deliver sample1
//                    .lineToY(-16)
//                        .setTangent(Math.toRadians(0))
//                    .lineToX(39)
//                        .setTangent(Math.toRadians(90))
//                    .lineToY(-57)  // deliver sample2
//                    .lineToY(-16)
//                        .setTangent(Math.toRadians(0))
//                    .lineToX(43)
//                        .setTangent(Math.toRadians(90))
//                    .lineToY(-57)  // deliver sample3
//                    .lineToY(-60)  // grab spec1
//                    .splineToConstantHeading(new Vector2d(0, -32), Math.toRadians(90))  // score spec1
//                    .splineToConstantHeading(new Vector2d(30, -57), Math.toRadians(90))  // pre-pickup spec2
//                        .lineToY(-60)  // grab spec2
//                        .splineToConstantHeading(new Vector2d(0, -32), Math.toRadians(90))  // score spec2
//                        .splineToConstantHeading(new Vector2d(30, -57), Math.toRadians(90)) // pre-pickup spec2
//                        .lineToY(-60) // grab spec 2
//                        .splineToConstantHeading(new Vector2d(0, -32), Math.toRadians(90)) // score spec2
//                        .splineToConstantHeading(new Vector2d(30, -57), Math.toRadians(90)) // pre-pickup spec3
//                        .lineToY(-60)  // grab spec3
//                        .splineToConstantHeading(new Vector2d(0, -32), Math.toRadians(90))  // score spec3
                        .setTangent(-90)
                        .strafeToSplineHeading(new Vector2d(0, -36), Math.toRadians(-90))
                        .strafeToSplineHeading(new Vector2d(27, -39), Math.toRadians(-90))
                        //   .splineToLinearHeading(new Pose2d(35, -16, Math.toRadians(-90)), Math.toRadians(-90))
                        .strafeToSplineHeading(new Vector2d(27, -16), Math.toRadians(-90))
                        .strafeToSplineHeading(new Vector2d(35, -16), Math.toRadians(-90))
                        .strafeToSplineHeading(new Vector2d(35, -57), Math.toRadians(-90))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(40, -16, Math.toRadians(-90)), Math.toRadians(-90))
//                        .strafeToSplineHeading(new Vector2d(35, -16), Math.toRadians(-90))
//                        .strafeToSplineHeading(new Vector2d(40, -16), Math.toRadians(-90))
//                        .strafeToSplineHeading(new Vector2d(40, -57), Math.toRadians(-90))
//                        .strafeToSplineHeading(new Vector2d(40, -16), Math.toRadians(-90))
//                        .strafeToSplineHeading(new Vector2d(45, -16), Math.toRadians(-90))
//                        .strafeToSplineHeading(new Vector2d(45, -57), Math.toRadians(-90))



                        .build());


    }
}