package OLD;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Hardware.HardwareNoDriveTrainRobot;
import RR.MecanumDrive;

//@Config
//@Autonomous(name = "RR Auto Red Basket v1.1", group = "Auto")

public final class AutoRedBasketRR extends LinearOpMode {
    public static double DISTANCE = 64;



    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-32, -62, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();



        waitForStart();



//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(-32, -62, Math.toRadians(0)))
//                //    .lineToY(-33)
//                        .setTangent(45)
//                    .splineToLinearHeading(new Pose2d(-54, -59, Math.toRadians(45)), Math.toRadians(0))// score preload
//                    .build());

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-32,-62,Math.toRadians(0)))
                        .setTangent(45)
                        .strafeToSplineHeading(new Vector2d(-54, -59), Math.toRadians(45))

                   //     .setTangent(0)
                        //outtake preload
                        .turnTo(Math.toRadians(79))
                        //intake
                        .turnTo(Math.toRadians(45))
                        //outtake
                        .turnTo(Math.toRadians(100))
                        //intake
                        .turnTo(Math.toRadians(45))
                        //outtake
                        .turnTo(Math.toRadians(120))
                        //intake
                        .turnTo(Math.toRadians(45))
                        //outtake
                        .build());



    }

    /**to implementing this:
     *         Actions.runBlocking(
     *                 new SequentialAction(
     *                         trajectoryActionChosen,
     *                         AutoOuttakeSliderHighBasket(),
     *                         trajectory....
     *                         .waitSeconds(3)
     *                         AutoOuttakeSliderHighBasket(),
     *                         trajectoryActionCloseOut
     *                 )
     *         );
     *
     */
    public class AutoOuttakeSlider  {
        HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();
        public class AutoOuttakeSliderHighBasket implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    autoRobot.Outtake.leftSlideSetPositionPower(3400,0.5);
                    autoRobot.Outtake.rightSlideSetPositionPower(3400,0.5);
                    initialized = true;
                }
                double positionOuttakeLeftSlide = autoRobot.Outtake.outtakeLeftSlide.getCurrentPosition();
                packet.put("Outtake slider-left position", positionOuttakeLeftSlide);
                if (positionOuttakeLeftSlide < 3400.0) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action AutoOuttakeSliderHighBasket() {
            return new AutoOuttakeSliderHighBasket();
        }


        public class AutoOuttakeSliderReadyPosition implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    autoRobot.Outtake.leftSlideSetPositionPower(500,0.5);
                    autoRobot.Outtake.rightSlideSetPositionPower(500,0.5);
                    initialized = true;
                }
                double positionOuttakeLeftSlide = autoRobot.Outtake.outtakeLeftSlide.getCurrentPosition();
                packet.put("Outtake slider-left position", positionOuttakeLeftSlide);

                if (positionOuttakeLeftSlide > 500.0) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action AutoOuttakeSliderReadyPosition(){
            return new AutoOuttakeSliderReadyPosition();
        }
    }




}