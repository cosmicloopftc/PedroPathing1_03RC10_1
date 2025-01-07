package RR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Hardware.HardwareNoDriveTrainRobot;


public class RRAuto_1RedBasket extends RRAutoCore {


    //TODO: setup initial position for all subsystems
    //    public static double autoEnd_SliderMotorPosition,
    //            autoEnd_Slider_ServoArmPosition;



//-------------------------------------------------------------------------------------------------
    @Override
    public void runOpMode() throws InterruptedException {
        String AllianceBasketOrSpecimen = "1RedBasket";
        Pose2d beginPose = new Pose2d(0, 0, 0);     //TODO: would overide this for each case

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();    //TODO: will this interfere with follower(hardwareMap)? in .init
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
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
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .build());


        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        RRAutoCoreTelemetryDuringteleOp();          //show robot drawing on FTC Dashboard //TODO: add in the AUTOcore method to transfer data to teleOp
        telemetryA.update();
    }


}

