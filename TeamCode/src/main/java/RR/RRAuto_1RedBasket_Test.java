package RR;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import Hardware.HardwareNoDriveTrainRobot;

@Config
@Autonomous(name = "RR_TEST 1Auto Red Basket v1.1", group = "Auto")
public class RRAuto_1RedBasket_Test extends RRAutoCore {


    //TODO: setup initial position for all subsystems
    //    public static double autoEnd_SliderMotorPosition,
    //            autoEnd_Slider_ServoArmPosition;

//    AutoOuttakeSliderAction autoOuttakeSliderAction = null;
    //HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();

    //-------------------------------------------------------------------------------------------------
    @Override
    public void runOpMode() throws InterruptedException {

        String AllianceBasketOrSpecimen = "1RedBasket";
        Pose2d beginPose = new Pose2d(-32, -62, Math.toRadians(0));     //TODO: would overide this for each case

//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        //autoRobot.init(hardwareMap);   //for all hardware except drivetrain.  note hardwareMap is default and part of FTC Robot Controller HardwareMap class
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
//        autoDebug(500, "Auto:Init", "DONE");
//
//        telemetryA.update();

        TrajectoryActionBuilder preScore = drive.actionBuilder(beginPose)
                .setTangent(45)
                .strafeToSplineHeading(new Vector2d(-54, -59), Math.toRadians(45))

      //          .stopAndAdd(new AutoOuttakeSliderAction(3400, 0.5))

                .waitSeconds(0);

        TrajectoryActionBuilder turnForSample3 = preScore.endTrajectory().fresh()
                .turnTo(Math.toRadians(79))
                .waitSeconds(0);

        TrajectoryActionBuilder turnToBasket3 = turnForSample3.endTrajectory().fresh()
                .turnTo(Math.toRadians(79))
                .waitSeconds(0);




        //*****LOOP wait for start, INIT LOOP***********************************************************
        while (!isStarted() && !isStopRequested()) {
   //         RRAutoCoreInitLoop();
        }

        waitForStart();
        if (isStopRequested()) {//return;
            ////TODO:  methods to constantly write into into our AUTOstorage class to transfer to Teleop
            //
            sleep(1000);
        }


        //*********STARTING MAIN PROGRAM****************************************************************
  //      opmodeTimer.resetTimer();






//        autoRobot.Outtake.slidersOnlyHighBasket();
//        Actions.runBlocking(
//                new SequentialAction(
//                        preScore.build()
//                )
//        );
//        sleep(3000);
//        autoRobot.Outtake.slidersOnlyGroundPosition();
//        Actions.runBlocking(
//                new SequentialAction(
//                        turnForSample3.build()
//                )
//        );



        Actions.runBlocking(
                new SequentialAction(
                        preScore.build(),
                        autoOuttakeSliderAction(3200,1)
                )
        );



        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
//        RRAutoCoreTelemetryDuringteleOp();          //show robot drawing on FTC Dashboard //TODO: add in the AUTOcore method to transfer data to teleOp
//        telemetryA.update();
        telemetry.update();
        sleep(100000);
    }


//    public class AutoOuttakeSliderAction implements Action {
//        private boolean initialized = false;
//        int desirePosition;
//        double desiredPower;
//        ElapsedTime timer;
//
//        public   AutoOuttakeSliderAction(int position, double power) {
//            this.desirePosition = position;this.desiredPower = power;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!initialized) {
//                timer = new ElapsedTime();
//                autoRobot.Outtake.leftSlideSetPositionPower(desirePosition,desiredPower);
//                autoRobot.Outtake.rightSlideSetPositionPower(desirePosition,desiredPower);
//                initialized = true;
//            }
//            double positionOuttakeLeftSlide = autoRobot.Outtake.outtakeLeftSlide.getCurrentPosition();
//            packet.put("Outtake slider-left position", positionOuttakeLeftSlide);
//            if (Math.abs((positionOuttakeLeftSlide - desirePosition)) > 20) {
//                return true;
//            } else {
//                return false;
//            }
//        }
//    }
//    public Action autoOuttakeSliderAction(int position, double power){
//        return new AutoOuttakeSliderAction(position, power);
//    }
}