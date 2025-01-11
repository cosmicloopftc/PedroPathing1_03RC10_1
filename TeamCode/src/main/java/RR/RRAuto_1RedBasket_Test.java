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
@Autonomous(name = "RR_TEST 1Auto Red Basket v1.1", group = "Auto")
public class RRAuto_1RedBasket_Test extends RRAutoCore {


    //TODO: setup initial position for all subsystems
    //    public static double autoEnd_SliderMotorPosition,
    //            autoEnd_Slider_ServoArmPosition;



//-------------------------------------------------------------------------------------------------
    @Override
    public void runOpMode() throws InterruptedException {
        //AutoOuttakeSlider autoOuttakeSlider = new  AutoOuttakeSlider();

        String AllianceBasketOrSpecimen = "1RedBasket";
        Pose2d beginPose = new Pose2d(-32, -62, Math.toRadians(0));     //TODO: would overide this for each case

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();    //TODO: will this interfere with follower(hardwareMap)? in .init
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        autoDebug(500, "Auto:Init", "DONE");

        telemetryA.update();

        TrajectoryActionBuilder preScore = drive.actionBuilder(beginPose)
                .setTangent(45)
                .strafeToSplineHeading(new Vector2d(-54, -59), Math.toRadians(45))
                .waitSeconds(0)
                .stopAndAdd(new AutoOuttakeSliderAction(3400, 0.5));







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
                        preScore.build()


                )
        );




//        Actions.runBlocking(
//                new SequentialAction(
//                        preScore.build(),
//
//
//                )
//        );



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

