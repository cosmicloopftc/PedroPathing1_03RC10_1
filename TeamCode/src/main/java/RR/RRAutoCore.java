package RR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Hardware.HardwareNoDriveTrainRobot;

public class RRAutoCore extends LinearOpMode {
    int debugLevel = 499;

    private Telemetry telemetryA;
    private HuskyLens huskyLens;
    private Timer pathTimer, actionTimer, opmodeTimer;


//TODO: setup initial position for all subsystems
//    public static double autoEnd_SliderMotorPosition,
//            autoEnd_Slider_ServoArmPosition;


    @Override
    public void runOpMode() throws InterruptedException {
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();    //TODO: will this interfere with follower(hardwareMap)? in .init


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        Pose2d beginPose = new Pose2d(0, 0, 0);

        autoDebug(500, "Auto:Init", "DONE");

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

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
                telemetry.update();

    TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);










    //Debugging messages
    void autoDebug(int myLevel, String myName, String myMessage) {
        if (debugLevel > myLevel)  {
            telemetryA.addData("**DEBUG**: " + myName, myMessage);
            telemetryA.update();
        }
        if ((1000 > myLevel) || (debugLevel > myLevel)) {
            RobotLog.i("**DEBUG** == " + myName + ": " + myMessage);
        }
    }

    //Log messages that are always shown
    void autoLog(String myName, String myMessage){
        telemetryA.addData(myName, myMessage);
        telemetryA.update();
        RobotLog.i("LOG == " + myName + ": " + myMessage);
    }
}