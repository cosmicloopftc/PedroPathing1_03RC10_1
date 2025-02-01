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

import Hardware.HardwareNoDriveTrainRobot;

/**2/1/2025:  add parallel action--up to first specimen collect from wall.
 *
 *
 *
 *
 */


@Config
@Autonomous(name = "RR_2AutoRedSpecimen_Push_ParallelMove v1.1", group = "Auto")
public class RR_2AutoRedSpecimen_PushTestParallelMove extends LinearOpMode {


    //TODO: setup initial position for all subsystems
    //    public static double autoEnd_SliderMotorPosition,
    //            autoEnd_Slider_ServoArmPosition;


    HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();    //TODO: will this interfere with follower(hardwareMap)? in .init


    //-------------------------------------------------------------------------------------------------
    @Override
    public void runOpMode() throws InterruptedException {
        double sliderPower = 0.5;

        AutoOuttakeSliderAction autoOuttakeSliderAction = null;
        autoRobot.init(hardwareMap);   //for all hardware except drivetrain.  note hardwareMap is default and part of FTC Robot Controller HardwareMap class

        String AllianceBasketOrSpecimen = "1RedSpecimen";
 //       Pose2d beginPose = new Pose2d(38, -63, Math.toRadians(-90));    //TODO: would overide this for each case
        Pose2d beginPose = new Pose2d(7.5, -62.85, Math.toRadians(-90));

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
        VelConstraint velMedium = new TranslationalVelConstraint(60);
        VelConstraint velFast = new TranslationalVelConstraint(90);
        AccelConstraint accSlow = new ProfileAccelConstraint(-15,15);
        AccelConstraint accMedium = new ProfileAccelConstraint(-60,60);
        AccelConstraint accFast = new ProfileAccelConstraint(-90,90);

        /**  .lineToX(24.5,velSlow,accSlow)        //example on how to use these in drive.actionBuilder */


        TrajectoryActionBuilder preloadScore;
        preloadScore = drive.actionBuilder(beginPose)
                .lineToYConstantHeading(-37,velFast, accMedium)             //start and move to submersible pole
                .stopAndAdd(new AutoClawAction(0.7,0.5))        //open claw
                .lineToYConstantHeading(-43, velFast, accMedium);           //1st move back from submersible pole

        TrajectoryActionBuilder preloadMoveBack= preloadScore.endTrajectory().fresh()
                //claw open separately before this
                //.lineToYConstantHeading(-43, velFast, accMedium)
                //.stopAndAdd(new AutoOuttakeArmAxonAction(0.4, 0))
                .splineToLinearHeading(new Pose2d(38,-40, Math.toRadians(-90)),
                        Math.toRadians(-90), velFast, accMedium);



        TrajectoryActionBuilder gotoSample4 = preloadMoveBack.endTrajectory().fresh()
                .lineToYConstantHeading(-20, velFast, accMedium)
                .splineToLinearHeading(new Pose2d(47,-14, Math.toRadians(-90)),
                        Math.toRadians(-90), velFast, accMedium);

        TrajectoryActionBuilder pushSample4HomeThenToSample5 = gotoSample4.endTrajectory().fresh()
                .lineToYConstantHeading(-52, velFast, accMedium)
                .lineToYConstantHeading(-20, velFast, accMedium)
                .splineToLinearHeading(new Pose2d(51,-8, Math.toRadians(-90)),
                        Math.toRadians(-90), velFast, accMedium);

        TrajectoryActionBuilder pushSample5HomeThenToSample6 = gotoSample4.endTrajectory().fresh()
                .lineToYConstantHeading(-52, velFast, accMedium)
                .lineToYConstantHeading(-20, velFast, accMedium)
                .splineToLinearHeading(new Pose2d(58,-16, Math.toRadians(-90)),
                        Math.toRadians(-90), velFast, accMedium);

        TrajectoryActionBuilder pushSample6Home = pushSample5HomeThenToSample6.endTrajectory().fresh()
                .lineToYConstantHeading(-54, velFast, accMedium);





        TrajectoryActionBuilder collectSpec1 = pushSample6Home.endTrajectory().fresh()
                //below already done when moving back from submersible---during preloadMoveBack
                //.stopAndAdd(new AutoOuttakeSliderAction(0, 1))
                //.stopAndAdd(new AutoouttakeExtensionAction(0.85))  //extend arm (need to this to the place after preloadscore
                //.stopAndAdd(new AutoOuttakeArmAxonAction(0.28,0))   //extend arm (need to this to the place after preloadscore

                .setTangent(-90)
                .lineToY(-65);
                //below already done when moving to submersible
                //.stopAndAdd(new AutoClawAction(1, 0.5))               //close claw
                //.stopAndAdd(new AutoOuttakeSliderAction(1240, 1))        //move slider up
                //.stopAndAdd(new AutoouttakeExtensionAction(1))                  //bring arm in
                //.stopAndAdd(new AutoOuttakeArmAxonAction(0.9,0));  //rotate to specimen score position

        TrajectoryActionBuilder firstScoreSpec =  collectSpec1.endTrajectory().fresh()
                .lineToYConstantHeading(-62)
                .splineToConstantHeading(new Vector2d(5,-40),Math.toRadians(90))
                .lineToYConstantHeading(-37);

        TrajectoryActionBuilder collectSpec2 = firstScoreSpec.endTrajectory().fresh()
                .lineToYConstantHeading(-45)
                .stopAndAdd(new AutoOuttakeArmAxonAction(0.4, 0))
                .stopAndAdd(new AutoOuttakeSliderAction(0, 1))

                .stopAndAdd(new AutoouttakeExtensionAction(0.85))  //extend arm (need to this to the place after preloadscore
                .stopAndAdd(new AutoOuttakeArmAxonAction(0.28,0))   //extend arm (need to this to the place after preloadscore
                .lineToYConstantHeading(-45)
                .splineToConstantHeading(new Vector2d(38,-60),Math.toRadians(270))
                .lineToYConstantHeading(-65)

                .stopAndAdd(new AutoClawAction(1, 0.5))               //close claw

                .stopAndAdd(new AutoOuttakeSliderAction(1240, 1))         //move slider up
                .stopAndAdd(new AutoouttakeExtensionAction(1))            //bring arm in
                .stopAndAdd(new AutoOuttakeArmAxonAction(0.9,0));

        TrajectoryActionBuilder secondScoreSpec = collectSpec2.endTrajectory().fresh()
                .lineToYConstantHeading(-62)
                .splineToConstantHeading(new Vector2d(5,-40),Math.toRadians(90))
                .lineToYConstantHeading(-37);

        TrajectoryActionBuilder collectSpec3 = secondScoreSpec.endTrajectory().fresh()
                .lineToYConstantHeading(-45)
                .stopAndAdd(new AutoOuttakeArmAxonAction(0.4, 0))
                .stopAndAdd(new AutoOuttakeSliderAction(0, 1))

                .stopAndAdd(new AutoouttakeExtensionAction(0.85))  //extend arm (need to this to the place after preloadscore
                .stopAndAdd(new AutoOuttakeArmAxonAction(0.28,0))   //extend arm (need to this to the place after preloadscore
                .lineToYConstantHeading(-45)
                .splineToConstantHeading(new Vector2d(38,-60),Math.toRadians(270))
                .lineToYConstantHeading(-65)

                .stopAndAdd(new AutoClawAction(1, 0.5))               //close claw

                .stopAndAdd(new AutoOuttakeSliderAction(1240, 1))         //move slider up
                .stopAndAdd(new AutoouttakeExtensionAction(1))            //bring arm in
                .stopAndAdd(new AutoOuttakeArmAxonAction(0.9,0));

        TrajectoryActionBuilder thirdScoreSpec = collectSpec3.endTrajectory().fresh()
                .lineToYConstantHeading(-62)
                .splineToConstantHeading(new Vector2d(5,-40),Math.toRadians(90))
                .lineToYConstantHeading(-37 )

                .stopAndAdd(autoClawAction(0.7,0))
                .lineToYConstantHeading(-43)
                .stopAndAdd(new AutoOuttakeArmAxonAction(0.4, 0))
                .stopAndAdd(new AutoOuttakeSliderAction(0, 1));






        //*****LOOP wait for start, INIT LOOP***********************************************************
        while (!isStarted() && !isStopRequested()) {
            //         RRAutoCoreInitLoop();
            /**PLACE Huskylens code here for testing purpose*/

            /**then PLACE LED reading here for reading purpose*/

            autoRobot.Outtake.extendIN();
            autoRobot.Outtake.outtakeArmAxon.setPosition(0.28);
            autoRobot.Outtake.closeClaw();
      //      autoRobot.Intake.intakeINSIDEBOT();

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
                        //Initialize
                        autoOuttakeArmAxonAction(0.9,0),        //rotate outtake arm to scoring position
                        autoOuttakeSliderAction(1240,1),      //raise outtake  slider to scoring position
                        preloadScore.build(),               //move forward to submersible; **open claw; 1st move back from submersible

                        //autoClawAction(0.7,0),                  //open claw
                        new ParallelAction(
                                autoOuttakeArmAxonAction(0.28, 0),      //TODO: position? rotate outtake arm to position to grab specimen on wall
                                autoouttakeExtensionAction(0.85),           //TODO: position?  extend out arm out (need to this to the place after preloadscore
                                autoOuttakeSliderAction(0, 1),               //TODO: ?outtake slider position to pickup specimen from wall
                                preloadMoveBack.build()
                            ),



                        gotoSample4.build(),
                        //pushSample4HomeThenToSample5.build(),
                        pushSample5HomeThenToSample6.build(),
                        pushSample6Home.build(),


                        collectSpec1.build(),
                        autoClawAction(1, 0.5),                     //close claw
                        new ParallelAction(
                                autoOuttakeSliderAction(1240, 1),        //move slider up
                                autoouttakeExtensionAction(1),                 //bring arm in
                                autoOuttakeArmAxonAction(0.9,0),  //rotate to specimen score position
                                firstScoreSpec.build()                                  //goto submersible pole
                            ),




                        autoClawAction(0.7,0),
                        collectSpec2.build(),
                        secondScoreSpec.build(),
                        autoClawAction(0.7,0),
                        collectSpec3.build(),
                        thirdScoreSpec.build()




                        /**
                        intakeSample1.build(),
                        deliverSample1.build(),
                        intakeSample2.build(),
                        deliverSample2.build(),
                        collectSpec1.build(),
                        scoreSpec1.build(),
                        collectSpec2.build()

                        intakeSample2.build(),
                        deliverSample2.build()

                        autoOuttakeSliderHighBasketAction(),
                        autoOuttakeArmAxonAction(0.75),
                        autoClawAction(0.7),

                        //move away from chamber
                        //afterScore.build(),
                        autoIntakeSliderAction(10, sliderPower),
                        autoIntakeServoAxon(0.9),
                        autoIntakeSliderAction(0,sliderPower),
                        autoIntakeServoAxon(0.6),
                        autoClawAction(0.32)
                        //deliver samples
                        */


                        /**
                         intakeSample1.build(),
                         deliverSample1.build(),
                         intakeSample2.build(),
                         deliverSample2.build(),
                         collectSpec1.build(),
                         scoreSpec1.build(),
                         collectSpec2.build(),
                         scoreSpec2.build(),
                         collectSpec3.build(),
                         scoreSpec3.build()  */

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
        sleep(100000);
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


    public class AutoIntakeSpinerAction implements Action {
        private boolean initialized = false;
        double power;
        ElapsedTime timer;
        double pauseTimeSec;
        public   AutoIntakeSpinerAction(double power, double pauseTimeSec) {

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
        return new AutoIntakeSpinerAction(power, pauseTimeSec);
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




////Debugging messages
//void autoDebug(int myLevel, String myName, String myMessage) {
//    if (debugLevel > myLevel)  {
//        telemetryA.addData("**DEBUG**: " + myName, myMessage);
//        telemetryA.update();
//    }
//    if ((1000 > myLevel) || (debugLevel > myLevel)) {
//        RobotLog.i("**DEBUG** == " + myName + ": " + myMessage);
//    }
//}
//
////Log messages that are always shown
//void autoLog(String myName, String myMessage){
//    telemetryA.addData(myName, myMessage);
//    telemetryA.update();
//    RobotLog.i("LOG == " + myName + ": " + myMessage);
//}