package RR;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import Hardware.HardwareNoDriveTrainRobot;

//@Config
//@Autonomous(name = "RR_1AutoRedBasket v1.1", group = "Auto")
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

        double readyPosition = 0.43;
        double grabPosition = 0.34;
        double intakeAxonPosition = 0.65;

        /**  .lineToX(24.5,velSlow,accSlow)        //example on how to use these in drive.actionBuilder */

        TrajectoryActionBuilder preScore = drive.actionBuilder(beginPose)
                .setTangent(45)
                .waitSeconds(0.25)
                .strafeToSplineHeading(new Vector2d(-56, -55), Math.toRadians(45));

        TrajectoryActionBuilder grabPose = preScore.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-56, -43), Math.toRadians(69));

        TrajectoryActionBuilder turnToBasket3 = grabPose.endTrajectory().fresh()
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(-55.5, -54.5), Math.toRadians(45));

        TrajectoryActionBuilder grabPose2 = turnToBasket3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-56, -43), Math.toRadians(95));

        TrajectoryActionBuilder grabPose3 = turnToBasket3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-56, -43), Math.toRadians(124));

        TrajectoryActionBuilder submersible = turnToBasket3.endTrajectory().fresh()
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(-22.5, 0, 0), Math.toRadians(35));

        TrajectoryActionBuilder turnToBasket2 = submersible.endTrajectory().fresh()
                .setTangent(45)
                .strafeToConstantHeading(new Vector2d(-25, 0))
                .strafeToSplineHeading(new Vector2d(-56, -55), Math.toRadians(45));



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
//                        preScore.build(),
                new SequentialAction(
                        //Raise Outtake slider and Move Robot to basket
                        autoOuttakeSliderHighBasketAction(),
                        autoOuttakeArmAxonAction(0.75, 0),
                        preScore.build(),


                        /** Sample 1*/
                        autoClawAction(0.7, 0.15),
                        autoOuttakeArmAxonAction(readyPosition, 0),
                        autoIntakeServoAxonAction(0.97),
                        autoIntakeSpiner(-1, 0),
                        new ParallelAction(
                            autoOuttakeSliderAction(0, 1),
                            //move to first sample
                            grabPose.build()
                        ),
                        autoIntakeSliderAction(220, sliderPower, 0.5),

                        /*NEXT STEP*/
                        autoIntakeSliderAction(1, sliderPower, 0),
                        autoIntakeServoAxonAction(intakeAxonPosition),
                        autoIntakeSpiner(0, 0.5),
                        autoOuttakeArmAxonAction(grabPosition, 0.5),
                        autoClawAction(1, 0.25),
                        new ParallelAction(
                        autoOuttakeSliderHighBasketAction(),
                        turnToBasket3.build()
                                ),
                        autoOuttakeArmAxonAction(0.77, 0.5),

                        /** Sample 2*/
                        autoClawAction(0.7, 0.15),
                        autoOuttakeArmAxonAction(readyPosition, 0),
                        autoIntakeServoAxonAction(0.97),
                        autoIntakeSpiner(-1, 0),
                        //move to first sample
                        new ParallelAction(
                                autoOuttakeSliderAction(0, 1),
                                //move to first sample
                                grabPose2.build()
                        ),
                        autoIntakeSliderAction(220, sliderPower, 0.5),

                        /*NEXT STEP*/
                        autoIntakeSliderAction(1, sliderPower, 0),
                        autoIntakeServoAxonAction(intakeAxonPosition),
                        autoIntakeSpiner(0, 0.5),
                        autoOuttakeArmAxonAction(grabPosition, 0.5),
                        autoClawAction(1, 0.25),
                        new ParallelAction(
                                autoOuttakeSliderHighBasketAction(),
                                turnToBasket3.build()
                        ),
                        autoOuttakeArmAxonAction(0.77, 0.5),

                        /**Sample 3*/
                        autoClawAction(0.7, 0.15),
                        autoOuttakeArmAxonAction(readyPosition, 0),

                        autoIntakeServoAxonAction(0.97),
                        autoIntakeSpiner(-1, 0),
                        //move to first sample
                        new ParallelAction(
                                autoOuttakeSliderAction(0, 1),
                                //move to first sample
                                grabPose3.build()
                        ),
                        autoIntakeSliderAction(240, sliderPower, 0.5),

                        /*NEXT STEP*/
                        autoIntakeSliderAction(1, sliderPower, 0),
                        autoIntakeServoAxonAction(intakeAxonPosition),
                        autoIntakeSpiner(0, 0.5),
                        autoOuttakeArmAxonAction(grabPosition, 0.5),
                        autoClawAction(1, 0.25),
                        new ParallelAction(
                                autoOuttakeSliderHighBasketAction(),
                                turnToBasket3.build()
                        ),
                        autoOuttakeArmAxonAction(0.77, 0.5),
                        autoClawAction(0.7, 0.15),

                        //End of V1
                        autoOuttakeArmAxonAction(readyPosition, 0),
                        autoOuttakeSliderAction(1, 1),

                        //Start of V2
                        submersible.build(),
                        autoIntakeServoAxonAction(0.97),
                        autoIntakeSpiner(-1, 0),
                        autoIntakeSliderAction(220, sliderPower, 0.5),

                        //Move to basket
                        autoIntakeSliderAction(1, sliderPower, 0),
                        new ParallelAction(
                                turnToBasket2.build(),
                                new SequentialAction(
                                autoIntakeServoAxonAction(intakeAxonPosition),
                                autoIntakeSpiner(0, 0.5),
                                autoOuttakeArmAxonAction(grabPosition, 0.5),
                                autoClawAction(1, 0.25),
                                autoOuttakeSliderHighBasketAction(),
                                autoOuttakeArmAxonAction(0.75, 0.25)
                                )
                        ),

                        //Final
                        autoClawAction(0.7, 0.15),
                        autoOuttakeArmAxonAction(readyPosition, 0),
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


        /**TRANSFER SUBSYSTEM positions at end of AUTO TO TELEOP */
        AUTOstorageConstant.AllianceBasketOrSpecimen = AllianceBasketOrSpecimen;
        //Intake servo and motor position position:
        AUTOstorageConstant.autoEnd_Intake_intakeSlides_MotorPosition = autoRobot.Intake.intakeSlides.getCurrentPosition();
        AUTOstorageConstant.autoEnd_Intake_intakeServoAxon_ServoPosition = autoRobot.Intake.intakeServoAxon.getPosition();  //TODO: confirm name
        //Outtake servo and motor position:
        AUTOstorageConstant.autoEnd_Outtake_outtakeLeftSlide_MotorPosition = autoRobot.Outtake.outtakeLeftSlide.getCurrentPosition();
        AUTOstorageConstant.autoEnd_Outtake_outtakeRightSlide_MotorPosition = autoRobot.Outtake.outtakeRightSlide.getCurrentPosition();
        AUTOstorageConstant.autoEnd_Outtake_outtakeArmAxon_ServoPosition = autoRobot.Outtake.outtakeArmAxon.getPosition();  //TODO: confirm name
        AUTOstorageConstant.autoEnd_Outtake_claw_ServoPosition = autoRobot.Outtake.claw.getPosition();
        //Drivetrain: Pose and heading
        AUTOstorageConstant.autoEndheadingIMU_yawDEG = autoRobot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);;
        AUTOstorageConstant.autoPoseEnd =  poseEnd;
        AUTOstorageConstant.autoEndX = poseEnd.position.x;
        AUTOstorageConstant.autoEndY = poseEnd.position.y;
        AUTOstorageConstant.autoEndHeadingDEG = Math.toDegrees(poseEnd.heading.toDouble());





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