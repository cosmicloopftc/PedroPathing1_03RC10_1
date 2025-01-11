package RR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Hardware.HardwareNoDriveTrainRobot;

@Config
@Autonomous(name = "RR_TEST 2Auto Red Specimen v1.1", group = "Auto")
public class RRAuto_2RedSpecimen_Test extends RRAutoCore {


    //TODO: setup initial position for all subsystems
    //    public static double autoEnd_SliderMotorPosition,
    //            autoEnd_Slider_ServoArmPosition;



//-------------------------------------------------------------------------------------------------
    @Override
    public void runOpMode() throws InterruptedException {
        //AutoOuttakeSlider autoOuttakeSlider = new  AutoOuttakeSlider();
        //AutoIntakeSlider autoIntakeSlider = new  AutoIntakeSlider();


        String AllianceBasketOrSpecimen = "2RedBasket";
        Pose2d beginPose = new Pose2d(7.5, -63.5, Math.toRadians(-90));     //TODO: would overide this for each case

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();    //TODO: will this interfere with follower(hardwareMap)? in .init
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);





        //--------------------------------------------------------------------------------
        TrajectoryActionBuilder scorePreloadSpecimen = drive.actionBuilder(beginPose)
                .lineToY(-33)
                .splineToConstantHeading(new Vector2d(0, -32), Math.toRadians(90))// score preload
                .waitSeconds(0)
                .stopAndAdd(new AutoOuttakeSliderAction(1300, 0.5))
                .stopAndAdd(new AutoOuttakeArmAxonAction(0.35));


        TrajectoryActionBuilder pushThreeSampleToWall = scorePreloadSpecimen.endTrajectory().fresh()
                //path

                .waitSeconds(0);

        TrajectoryActionBuilder pickSpecimen1atWall = pushThreeSampleToWall.endTrajectory().fresh()
                //path
                .waitSeconds(0);

        TrajectoryActionBuilder scoreSpecimen1 = pickSpecimen1atWall.endTrajectory().fresh()
                //path
                .waitSeconds(0);

        TrajectoryActionBuilder pickSpecimen2atWall = scoreSpecimen1.endTrajectory().fresh()
                //path
                .waitSeconds(0);

        TrajectoryActionBuilder scoreSpecimen2 = pickSpecimen2atWall.endTrajectory().fresh()
                //path
                .waitSeconds(0);

        TrajectoryActionBuilder pickSpecimen3atWall = scoreSpecimen2.endTrajectory().fresh()
                //path
                .waitSeconds(0);

        TrajectoryActionBuilder scoreSpecimen3 = pickSpecimen3atWall.endTrajectory().fresh()
                //path
                .waitSeconds(0);

        TrajectoryActionBuilder redSpecimenPark = scoreSpecimen3.endTrajectory().fresh()
                //path
                .waitSeconds(0);


        autoDebug(500, "Auto:Init", "DONE");
        telemetryA.update();

    //*****LOOP wait for start, INIT LOOP***********************************************************
        while (!isStarted() && !isStopRequested()) {
            RRAutoCoreInitLoop();
        }

        waitForStart();
        if (isStopRequested()) {//return;
            ////TODO:  methods to constantly write into into our AUTOstorage class to transfer to Teleop
            //
            sleep(1000);
        }


    //*********STARTING MAIN PROGRAM****************************************************************
        opmodeTimer.resetTimer();



        Actions.runBlocking(
                new SequentialAction(
                        scorePreloadSpecimen.build()


                        //autoOuttakeSlider.autoOuttakeSliderHighBasket(),
//                        pickSpecimen1atWall.build(),
//                        scoreSpecimen1.build(),
//                        pickSpecimen2atWall.build(),
//                        scoreSpecimen2.build(),
//                        pickSpecimen3atWall.build(),
//                        scoreSpecimen3.build(),
//                        redSpecimenPark.build()
                )
        );



        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        RRAutoCoreTelemetryDuringteleOp();          //show robot drawing on FTC Dashboard //TODO: add in the AUTOcore method to transfer data to teleOp
        telemetryA.update();
        sleep(10000);
    }


}

