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
        double sliderPower = 0.3;
        int sleepSeperation = 250;

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
                .waitSeconds(0.75)
                .strafeToSplineHeading(new Vector2d(-55, -58), Math.toRadians(45))
//                .strafeToConstantHeading(new Vector2d(-54, -59), velSlow,accSlow)

//                .autoOuttakeSliderHighBasketAction()
//                .autoOuttakeArmAxonAction(0.68)
//                .autoClawAction(0.7)
                //          .stopAndAdd(new AutoOuttakeSliderAction(3400, 0.5))
                .waitSeconds(0.75);

        TrajectoryActionBuilder waitPose = preScore.endTrajectory().fresh()
                .waitSeconds(0.4);

        TrajectoryActionBuilder turnForSample3 = preScore.endTrajectory().fresh()
                .waitSeconds(0.5)
                .turnTo(Math.toRadians(75));
//                .waitSeconds(4);

        TrajectoryActionBuilder turnForSample2 = preScore.endTrajectory().fresh()
//                .waitSeconds(5)
                .turnTo(Math.toRadians(95))
                .waitSeconds(0);

        TrajectoryActionBuilder turnForSample1 = preScore.endTrajectory().fresh()
//                .waitSeconds(5)
                .turnTo(Math.toRadians(110))
                .waitSeconds(0);

        TrajectoryActionBuilder turnToBasket3 = turnForSample3.endTrajectory().fresh()
                .waitSeconds(3)
                .strafeToSplineHeading(new Vector2d(-54, -59), Math.toRadians(45))
                .waitSeconds(1);

        TrajectoryActionBuilder grabPose = turnForSample3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-54, -45), Math.toRadians(57))
                .waitSeconds(0);

        TrajectoryActionBuilder grabWaitPose = grabPose.endTrajectory().fresh()
                .waitSeconds(0.5);
        TrajectoryActionBuilder grabWaitPose2 = turnForSample3.endTrajectory().fresh()
                .waitSeconds(1);
        TrajectoryActionBuilder grabWaitPose3 = turnForSample3.endTrajectory().fresh()
                .waitSeconds(0.5);
        TrajectoryActionBuilder grabWaitPose4 = turnForSample3.endTrajectory().fresh()
                .waitSeconds(2);
        TrajectoryActionBuilder grabWaitPose5 = turnForSample3.endTrajectory().fresh()
                .waitSeconds(1);



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
        opmodeTimer.resetTimer();


        Actions.runBlocking(
                new SequentialAction(
                        //Raise Outtake slider and Move Robot to basket
                        autoOuttakeSliderHighBasketAction(),
                        //SleepAction(sleepSeperation),
                        autoOuttakeArmAxonAction(0.75),
                        preScore.build(),

                        //turn to first sample
                        autoClawAction(0.7),
                        waitPose.build(),
                        autoOuttakeArmAxonAction(0.4),
                        autoOuttakeSliderAction(0, 1),
                        turnForSample3.build(),
                        autoIntakeServoAxonAction(0.94),
                        autoIntakeSpiner(-1),
                        waitPose.build(),
                        waitPose.build(),
                        waitPose.build(),
                        waitPose.build(),
                        //move to first sample
                        grabPose.build(),
                        turnForSample3.build(),
                        autoIntakeSliderAction(220, sliderPower),
                        grabWaitPose.build(),
                        grabWaitPose.build(),
                        grabWaitPose.build(),
                        grabWaitPose.build(),
                        grabWaitPose.build(),
                        grabWaitPose.build(),
                        grabWaitPose.build(),
                        grabWaitPose.build(),
                        /**NEXT STEP*/
                        autoIntakeSliderAction(1, sliderPower),
                        autoIntakeServoAxonAction(0.62),
                        autoIntakeSpiner(0),
                        grabWaitPose2.build(),
                        grabWaitPose2.build(),
                        autoOuttakeArmAxonAction(0.34),
                        grabWaitPose3.build(),
                        autoClawAction(1),
                        grabWaitPose4.build(),
                        autoOuttakeSliderHighBasketAction(),
//                        grabWaitPose5.build(),
                        turnToBasket3.build(),
                        autoClawAction(0.7)
//                        waitPose.build(),
//                        waitPose.build(),
//                        waitPose.build(),
//                        waitPose.build(),
//                        waitPose.build(),
//                        autoClawAction(1),
//                        autoOuttakeSliderHighBasketAction(),
//                        autoOuttakeArmAxonAction(0.75)
//                        turnToBasket3.build(),
//                        autoClawAction(0.7)
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

        public AutoOuttakeArmAxonAction(double position) {
            this.desirePosition = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Outtake.outtakeArmAxon.setPosition(desirePosition);
                initialized = true;
            }
            return false;
        }
    }

    public Action autoOuttakeArmAxonAction(double position) {
        return new AutoOuttakeArmAxonAction(position);
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

        public AutoClawAction(double position) {
            this.desirePosition = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Outtake.claw.setPosition(desirePosition);
                initialized = true;
            }
            return false;
        }
    }

    public Action autoClawAction(double position) {
        return new AutoClawAction(position);
    }


    /**
     * safe range for IntakeSlider MOTOR
     */
    public class AutoIntakeSliderAction implements Action {
        private boolean initialized = false;
        int desirePosition;
        double desiredPower;
        ElapsedTime timer;

        public AutoIntakeSliderAction(int position, double power) {
            this.desirePosition = position;
            this.desiredPower = power;
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
                return false;
            }

        }
    }

    public Action autoIntakeSliderAction(int position, double power) {
        return new AutoIntakeSliderAction(position, power);
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

        public   AutoIntakeSpiner(double power) {

            this.power = power;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Intake.intakeLeftWheel.setPower(power);
                autoRobot.Intake.intakeRightWheel.setPower(-power);
                initialized = true;
            }
            return false;
        }
    }
    //power:  positive = intake split OUT sample
    public Action autoIntakeSpiner(double power){
        return new AutoIntakeSpiner (power);
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