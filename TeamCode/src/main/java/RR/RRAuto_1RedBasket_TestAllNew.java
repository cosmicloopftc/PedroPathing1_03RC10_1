package RR;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import Hardware.HardwareNoDriveTrainRobot;
import RR.tuning.TuningOpModes;

@Config
@Autonomous(name = "RR_TEST_ALLNew 1Auto_RedBasket v1.1", group = "Auto")
public final class RRAuto_1RedBasket_TestAllNew extends LinearOpMode {
    HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-32, -62, Math.toRadians(0));     //TODO: would overide this for each case
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        autoRobot.init(hardwareMap);   //for all hardware except drivetrain.  note hardwareMap is default and part of FTC Robot Controller HardwareMap class

        TrajectoryActionBuilder preScore = drive.actionBuilder(beginPose)
                .setTangent(45)
                .strafeToSplineHeading(new Vector2d(-54, -59), Math.toRadians(45))
                //.stopAndAdd(new AutoOuttakeSliderAction(3400, 0.5))
                .waitSeconds(0);

        TrajectoryActionBuilder turnForSample3 = preScore.endTrajectory().fresh()
                .waitSeconds(5)
                .turnTo(Math.toRadians(79))
                .waitSeconds(0);

        TrajectoryActionBuilder turnToBasket3 = turnForSample3.endTrajectory().fresh()
                .turnTo(Math.toRadians(45))
                .waitSeconds(0);



     /**START RUN**********************************************************************************/
        waitForStart();


//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
//                        .splineTo(new Vector2d(0, 60), Math.PI)
//                        .build());

        Actions.runBlocking(
                new SequentialAction(
                        preScore.build(),
                        autoOuttakeSliderHighBasketAction(),
                        turnForSample3.build(),
                        autoOuttakeSliderAction(500,1),
                        turnToBasket3.build()


                )
        );
    }


    public class AutoOuttakeSliderAction implements Action {
        private boolean initialized = false;
        int desirePosition;
        double desiredPower;
        ElapsedTime timer;

        public   AutoOuttakeSliderAction(int position, double power) {
            this.desirePosition = position;this.desiredPower = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Outtake.leftSlideSetPositionPower(desirePosition,desiredPower);
                autoRobot.Outtake.rightSlideSetPositionPower(desirePosition,desiredPower);
                initialized = true;
            }
            double positionOuttakeLeftSlide = autoRobot.Outtake.outtakeLeftSlide.getCurrentPosition();
            packet.put("Outtake slider-left position", positionOuttakeLeftSlide);
            if (Math.abs((positionOuttakeLeftSlide - desirePosition)) > 20) {
                return true;
            } else {
                return false;
            }
        }
    }
    public Action autoOuttakeSliderAction(int position, double power){
        return new AutoOuttakeSliderAction(position, power);
    }


    public class AutoOuttakeSliderHighBasketAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            autoRobot.Outtake.slidersOnlyHighBasket();
            return false;
        }
    }
    public Action autoOuttakeSliderHighBasketAction() {
        return new AutoOuttakeSliderHighBasketAction();
    }


}