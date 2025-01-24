package RR;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.*;
import java.util.Arrays;

import Hardware.HardwareNoDriveTrainRobot;

@Config
@Autonomous(name = "RR_1AutoRedBasket v1.1", group = "Auto")
public class RR_1AutoRedBasket extends LinearOpMode {

    //TODO: setup initial position for all subsystems
    //    public static double autoEnd_SliderMotorPosition,
    //            autoEnd_Slider_ServoArmPosition;


    HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();    //TODO: will this interfere with follower(hardwareMap)? in .init


    //-------------------------------------------------------------------------------------------------
    @Override
    public void runOpMode() throws InterruptedException {
        double sliderPower = 0.35;

        AutoOuttakeSliderAction autoOuttakeSliderAction = null;
        autoRobot.init(hardwareMap);   //for all hardware except drivetrain.  note hardwareMap is default and part of FTC Robot Controller HardwareMap class

        String AllianceBasketOrSpecimen = "1RedBasket";
        Pose2d beginPose = new Pose2d(-32, -62, Math.toRadians(0));     //TODO: would overide this for each case

        int debugLevel = 499;
        Telemetry telemetryA;
        Timer pathTimer, actionTimer, opmodeTimer;
        Pose2d pose;

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        //autoDebug(500, "Auto:Init", "DONE");
        telemetryA.update();

        //adjust these settings as needed for use in the trajectory codes
        VelConstraint velSlow = new TranslationalVelConstraint(15);
        VelConstraint velFast = new TranslationalVelConstraint(45);
        AccelConstraint accSlow = new ProfileAccelConstraint(-15,15);
        AccelConstraint accFast = new ProfileAccelConstraint(-45,45);
        /**  .lineToX(24.5,velSlow,accSlow)        //example on how to use these in drive.actionBuilder */

        TrajectoryActionBuilder preScore = drive.actionBuilder(beginPose)
                .setTangent(45)
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(-56, -55), Math.toRadians(45));
//                .strafeToConstantHeading(new Vector2d(-54, -59), velSlow,accSlow)

//                .autoOuttakeSliderHighBasketAction()
//                .autoOuttakeArmAxonAction(0.68)
//                .autoClawAction(0.7)
        //          .stopAndAdd(new AutoOuttakeSliderAction(3400, 0.5))
//                .waitSeconds(0.25);

        TrajectoryActionBuilder waitPose = preScore.endTrajectory().fresh()
                .waitSeconds(0.4);

        TrajectoryActionBuilder turnForSample3 = preScore.endTrajectory().fresh()
                .waitSeconds(0.5)
                .turnTo(Math.toRadians(75));
//                .waitSeconds(4);

        TrajectoryActionBuilder turnForSample2 = preScore.endTrajectory().fresh()
//                .waitSeconds(5)
                .turnTo(Math.toRadians(85))
                .waitSeconds(0);

        TrajectoryActionBuilder turnForSample1 = preScore.endTrajectory().fresh()
//                .waitSeconds(5)
                .turnTo(Math.toRadians(110))
                .waitSeconds(0);

        TrajectoryActionBuilder grabPose = turnForSample3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-56, -43), Math.toRadians(69))
//                .strafeTo(new Vector2d(-54, -45))
                .waitSeconds(0);
        TrajectoryActionBuilder grabPose2 = turnForSample2.endTrajectory().fresh()
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(-56, -43), Math.toRadians(95))
//                .strafeTo(new Vector2d(-54, -45))
                .waitSeconds(0);
        TrajectoryActionBuilder grabPose3 = turnForSample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-56, -43), Math.toRadians(124))
//                .strafeTo(new Vector2d(-54, -45))
                .waitSeconds(0);

        TrajectoryActionBuilder grabWaitPose = grabPose.endTrajectory().fresh()
                .waitSeconds(0.5);

        TrajectoryActionBuilder turnToBasket3 = grabPose.endTrajectory().fresh()
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(-56, -55), Math.toRadians(45));
//                .waitSeconds(1);

        TrajectoryActionBuilder turnToBasket2 = turnForSample2.endTrajectory().fresh()
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(-52, -57), Math.toRadians(45));
//                .waitSeconds(1);

        TrajectoryActionBuilder turnToBasket1 = turnForSample1.endTrajectory().fresh()
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(-53.5, -54.5), Math.toRadians(45));
//                .waitSeconds(1);



        //*****LOOP wait for start, INIT LOOP***********************************************************
        while (!isStarted() && !isStopRequested()) {
            //         RRAutoCoreInitLoop();
            autoRobot.Outtake.extendIN();
            autoRobot.Outtake.outtakeArmAxon.setPosition(0.28);
            autoRobot.Outtake.closeClaw();
            autoRobot.Intake.intakeINSIDEBOT();

            telemetry.addLine("Initialized");
            telemetry.addData("Alliance Color/Mode: ", AllianceBasketOrSpecimen);
            telemetryA.addData("Starting X = ", beginPose.position.x);
            telemetryA.addData("Starting Y = ", beginPose.position.y);
            telemetryA.addData("Starting Heading (Degrees) = ", Math.toDegrees(beginPose.heading.toDouble()));
        }

        waitForStart();
        if (isStopRequested()) {//return;
            ////TODO:  methods to constantly write into into our AUTOstorage class to transfer to Teleop
            //
            sleep(1000);
        }


        //*********STARTING MAIN PROGRAM****************************************************************
        //      opmodeTimer.resetTimer();


        Actions.runBlocking(
                new SequentialAction(
                        //Raise Outtake slider and Move Robot to basket
                        autoOuttakeSliderHighBasketAction(),
                        //SleepAction(sleepSeperation),
                        autoOuttakeArmAxonAction(0.75, 0),
                        preScore.build(),

                        //turn to first sample
                        autoClawAction(0.7, 0.5),
                        autoOuttakeArmAxonAction(0.4, 0),
                        autoOuttakeSliderAction(1, 1),
//                        turnForSample3.build(),
                        autoIntakeServoAxonAction(0.97),
                        autoIntakeSpiner(-1, 0),
                        //move to first sample
                        grabPose.build(),
//                        turnForSample3.build(),
                        autoIntakeSliderAction(220, sliderPower, 0.5),

                        /*NEXT STEP*/
                        autoIntakeSliderAction(1, sliderPower, 0),
                        autoIntakeServoAxonAction(0.64),
                        autoIntakeSpiner(0, 0.75),
                        autoOuttakeArmAxonAction(0.33, 0.5),
                        autoClawAction(1, 0.25),
                        autoOuttakeSliderHighBasketAction(),
                        turnToBasket3.build(),
                        autoOuttakeArmAxonAction(0.75, 1),
                        autoClawAction(0.7, 0),

//                        /** Sample 2*/
                        autoClawAction(0.7, 0.5),
                        autoOuttakeArmAxonAction(0.4, 0),
                        autoOuttakeSliderAction(1, 1),
//                        turnForSample3.build(),
                        autoIntakeServoAxonAction(0.97),
                        autoIntakeSpiner(-1, 0),
                        //move to first sample
                        grabPose2.build(),
//                        turnForSample3.build(),
                        autoIntakeSliderAction(220, sliderPower, 0.5),

                        /*NEXT STEP*/
                        autoIntakeSliderAction(1, sliderPower, 0),
                        autoIntakeServoAxonAction(0.64),
                        autoIntakeSpiner(0, 0.75),
                        autoOuttakeArmAxonAction(0.32, 0.5),
                        autoClawAction(1, 0.25),
                        autoOuttakeSliderHighBasketAction(),
                        turnToBasket3.build(),
                        autoOuttakeArmAxonAction(0.75, 1),
                        autoClawAction(0.7, 0),
//
//                        /**Sample 3*/
                        autoClawAction(0.7, 0.5),
                        autoOuttakeArmAxonAction(0.4, 0),
                        autoOuttakeSliderAction(1, 1),
//                        turnForSample3.build(),
                        autoIntakeServoAxonAction(0.97),
                        autoIntakeSpiner(-1, 0),
                        //move to first sample
                        grabPose3.build(),
//                        turnForSample3.build(),
                        autoIntakeSliderAction(240, sliderPower, 0.5),

                        /*NEXT STEP*/
                        autoIntakeSliderAction(1, sliderPower, 0),
                        autoIntakeServoAxonAction(0.64),
                        autoIntakeSpiner(0, 0.75),
                        autoOuttakeArmAxonAction(0.33, 0.5),
                        autoClawAction(1, 0.25),
                        autoOuttakeSliderHighBasketAction(),
                        turnToBasket3.build(),
                        autoOuttakeArmAxonAction(0.75, 1),
                        autoClawAction(0.7, 0.5),

                        //final
                        autoOuttakeArmAxonAction(0.4, 0),
                        autoOuttakeSliderAction(0, 1)
                )
        );


        drive.updatePoseEstimate();
        Pose2d poseEnd = drive.localizer.getPose();
        telemetryA.addData("x", poseEnd.position.x);
        telemetryA.addData("y", poseEnd.position.y);
        telemetryA.addData("heading (deg)", Math.toDegrees(poseEnd.heading.toDouble()));
//        RRAutoCoreTelemetryDuringteleOp();          //show robot drawing on FTC Dashboard //TODO: add in the AUTOcore method to transfer data to teleOp
        telemetryA.update();
        //telemetry.update();
        sleep(300000);
    }

    private Action SleepAction(long milliseconds) {
        sleep(milliseconds);
        return null;
    }


    /**ACTION METHODS
     * to implementing actions with SequentialAction
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
     *to implement by
     *
     */

    /**
     * positions of OuttakeSlider: 0=ground, max/high basket=3400; high specimen bar=1300
     */
    public class AutoOuttakeSliderAction implements Action {
        private boolean initialized = false;
        int desirePosition;
        double desiredPower;
        ElapsedTime timer;

        public AutoOuttakeSliderAction(int position, double power) {
            this.desirePosition = position;
            this.desiredPower = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Outtake.leftSlideSetPositionPower(desirePosition, desiredPower);
                autoRobot.Outtake.rightSlideSetPositionPower(desirePosition, desiredPower);
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

    public Action autoOuttakeSliderAction(int position, double power) {
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


    /**
     * safe range for OuttakeArmAxon = 0.29 to 0.35 to 0.37 to 0.68 to 0.86
     */
    public class AutoOuttakeArmAxonAction implements Action {
        private boolean initialized = false;
        double desirePosition;
        ElapsedTime timer;
        double pauseTimeSec;

        public AutoOuttakeArmAxonAction(double position, double pauseTimeSec) {
            this.desirePosition = position;
            this.pauseTimeSec = pauseTimeSec;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Outtake.outtakeArmAxon.setPosition(desirePosition);
                initialized = true;
            }
            return timer.seconds() < pauseTimeSec;
        }
    }

    public Action autoOuttakeArmAxonAction(double position, double pauseTimeSec) {
        return new AutoOuttakeArmAxonAction(position, pauseTimeSec);
    }


    /**
     * safe range for  outtakeExtension =
     */
    public class AutoouttakeExtensionAction implements Action {
        private boolean initialized = false;
        double desirePosition;
        ElapsedTime timer;

        public AutoouttakeExtensionAction(double position) {
            this.desirePosition = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Outtake.outtakeExtension.setPosition(desirePosition);
                initialized = true;
            }
            return false;
        }
    }

    public Action autoouttakeExtensionAction(double position) {
        return new AutoouttakeExtensionAction(position);
    }


    /**
     * safe range for cLAW =
     */
    public class AutoClawAction implements Action {
        private boolean initialized = false;
        double desirePosition;
        ElapsedTime timer;
        double pauseTime;
        public AutoClawAction(double position, double pauseTime) {
            this.desirePosition = position;
            this.pauseTime = pauseTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Outtake.claw.setPosition(desirePosition);
                initialized = true;
            }
            return timer.seconds() < pauseTime;
        }
    }

    public Action autoClawAction(double position, double pauseTime) {
        return new AutoClawAction(position, pauseTime);
    }


    /**
     * safe range for IntakeSlider MOTOR
     */
    public class AutoIntakeSliderAction implements Action {
        private boolean initialized = false;
        int desirePosition;
        double desiredPower;
        ElapsedTime timer;
        double pauseTime;
        public AutoIntakeSliderAction(int position, double power, double pauseTime) {
            this.desirePosition = position;
            this.desiredPower = power;
            this.pauseTime = pauseTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Intake.intakeSlideSetPositionPower(desirePosition, desiredPower);
                initialized = true;
            }
            double positionIntakeSlide = autoRobot.Intake.intakeSlides.getCurrentPosition();
            packet.put("Intake slider position", positionIntakeSlide);
            if (Math.abs((positionIntakeSlide - desirePosition)) > 20) {
                return true;
            } else {
                return timer.seconds() < pauseTime;
            }

        }
    }

    public Action autoIntakeSliderAction(int position, double power, double pauseTime) {
        return new AutoIntakeSliderAction(position, power, pauseTime);
    }


    /**
     * safe range for intakeServoAxon =
     */
    public class AutoIntakeServoAxonAction implements Action {
        private boolean initialized = false;
        double desirePosition;
        ElapsedTime timer;

        public AutoIntakeServoAxonAction(double position) {
            this.desirePosition = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Intake.intakeServoAxon.setPosition(desirePosition);
                initialized = true;
            }
            return false;
        }
    }
    public Action autoIntakeServoAxonAction(double position) {

        telemetry.addLine("autoIntakeServoAxonAction");
        return new AutoIntakeServoAxonAction(position);
    }


    public class AutoIntakeSpiner implements Action {
        private boolean initialized = false;
        double power;
        ElapsedTime timer;
        double pauseTimeSec;
        public   AutoIntakeSpiner(double power, double pauseTimeSec) {

            this.power = power;
            this.pauseTimeSec = pauseTimeSec;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Intake.intakeLeftWheel.setPower(power);
                autoRobot.Intake.intakeRightWheel.setPower(-power);
                initialized = true;
            }
            return timer.seconds() < pauseTimeSec;
        }
    }
    //power:  positive = intake split OUT sample
    public Action autoIntakeSpiner(double power, double pauseTimeSec){
        return new AutoIntakeSpiner (power, pauseTimeSec);
    }

    public class AutoDiplayAction implements Action {
        private boolean initialized = false;
        String teleData;
        ElapsedTime timer;

        public AutoDiplayAction(String teleData) {
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                telemetry.addLine(teleData);
                telemetry.update();
                initialized = true;
            }
            return false;
        }
    }
    //power:  positive = intake split OUT sample
    public Action autoDiplayAction(String teleData){
        return new AutoDiplayAction(teleData);
    }

}
//    public Action cu
