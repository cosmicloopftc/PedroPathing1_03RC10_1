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

@Config
@Autonomous(name = "RR_2AutoRedSpecimen_PushTest v1.1", group = "Auto")
public class RR_2AutoRedSpecimen_PushTest extends LinearOpMode {


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
        Pose2d beginPose = new Pose2d(-7.5, -62.85, Math.toRadians(-90));    //TODO: would overide this for each case

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
        VelConstraint velFast = new TranslationalVelConstraint(60);
        AccelConstraint accSlow = new ProfileAccelConstraint(-15,15);
        AccelConstraint accFast = new ProfileAccelConstraint(-60,60);
        /**  .lineToX(24.5,velSlow,accSlow)        //example on how to use these in drive.actionBuilder */


        TrajectoryActionBuilder preloadScore;
        preloadScore = drive.actionBuilder(beginPose)
                //        .waitSeconds(0.5)
                .lineToYConstantHeading(-33,velFast, accFast);

        TrajectoryActionBuilder preloadMoveBack= preloadScore.endTrajectory().fresh()
                //claw open separately before this
                .lineToYConstantHeading(-40, velFast, accFast)
                .stopAndAdd(new AutoOuttakeArmAxonAction(0.4))          //rotate arm to inside robot
                .stopAndAdd(new AutoOuttakeSliderAction(0, 1))   //move outtake slider inside robot
                .splineToLinearHeading(new Pose2d(28,-43, Math.toRadians(-90)),
                        Math.toRadians(-90), velFast, accFast);

        //  .strafeToSplineHeading(new Vector2d(28, -43), Math.toRadians(60), velFast, accFast);

        TrajectoryActionBuilder gotoSample4 = preloadMoveBack.endTrajectory().fresh()
                .lineToYConstantHeading(-20)
                //next spline to behind Sample4 and ready to push it home.
                .splineToLinearHeading(new Pose2d(48,-5, Math.toRadians(-90)),
                        Math.toRadians(-90), velFast, accFast);

        TrajectoryActionBuilder pushSample4HomeThenToSample5 = gotoSample4.endTrajectory().fresh()
                .lineToYConstantHeading(-50)
                .lineToYConstantHeading(-20)
                .splineToLinearHeading(new Pose2d(48,-5, Math.toRadians(-90)),
                        Math.toRadians(-90), velFast, accFast);






        TrajectoryActionBuilder intakeSample1 = preloadMoveBack.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(28, -43), Math.toRadians(60), velFast, accFast)
                .stopAndAdd(new AutoIntakeServoAxonAction(0.94))
                .stopAndAdd(new AutoIntakeSpinerAction(-1))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoIntakeSliderAction(200, 0.3))
                .waitSeconds(0.1);
        //  .stopAndAdd(new AutoIntakeSpinerAction(0));
        //.stopAndAdd(new AutoIntakeSliderAction(, 0.5));

        TrajectoryActionBuilder deliverSample1 = intakeSample1.endTrajectory().fresh()
                .stopAndAdd(new AutoIntakeSliderAction(0, 1))
                .turnTo(Math.toRadians(-60))
                .stopAndAdd(new AutoIntakeSliderAction(200, 1))
                //     .waitSeconds(0.1)
                .stopAndAdd(new AutoIntakeSpinerAction(1))
                .waitSeconds(0.1)
                .stopAndAdd(new AutoIntakeSpinerAction(0))
                .waitSeconds(0.1)
//                .stopAndAdd(new AutoIntakeServoAxonAction(0.5))
                .stopAndAdd(new AutoIntakeSliderAction(0, 1));

        TrajectoryActionBuilder intakeSample2 = deliverSample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(35, -42), Math.toRadians(67))
                //       .turnTo(Math.toRadians(50))
//                .stopAndAdd(new AutoIntakeServoAxonAction(0.94))
                .stopAndAdd(new AutoIntakeSpinerAction(-1))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoIntakeSliderAction(200, 0.3));

        TrajectoryActionBuilder deliverSample2 = intakeSample2.endTrajectory().fresh()
                .stopAndAdd(new AutoIntakeSliderAction(0, 1))
                .turnTo(Math.toRadians(-50))
                .stopAndAdd(new AutoIntakeSliderAction(200, 1))
                .stopAndAdd(new AutoIntakeSpinerAction(1))
                .waitSeconds(0.1)
                .stopAndAdd(new AutoIntakeSpinerAction(0))
                .stopAndAdd(new AutoIntakeServoAxonAction(0.5))
                .stopAndAdd(new AutoIntakeSliderAction(0, 1));

        TrajectoryActionBuilder collectSpec1 = deliverSample2.endTrajectory().fresh()
                //  .strafeToSplineHeading(new Vector2d(40, -57), Math.toRadians(-90))
                //    .strafeToSplineHeading(new Vector2d(40, -60), Math.toRadians(-90));
                .turnTo(Math.toRadians(-90))
                .lineToY(-57)
                .stopAndAdd(new AutoouttakeExtensionAction(0.85))
                .stopAndAdd(new AutoOuttakeArmAxonAction(0.28))
                .lineToY(-63)
                .stopAndAdd(new AutoClawAction(1))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoOuttakeSliderAction(1700, 1))
                .stopAndAdd(new AutoouttakeExtensionAction(1))
                .stopAndAdd(new AutoOuttakeArmAxonAction(0.9));


        TrajectoryActionBuilder scoreSpec1 = collectSpec1.endTrajectory().fresh()
                .setReversed(true)
                .strafeToSplineHeading(new Vector2d(6, -40), Math.toRadians(-90), velFast, accFast)
                .waitSeconds(0.1)
                .lineToY(-33)
                .stopAndAdd(new AutoClawAction(0.7))
                .lineToY(-40)
                .stopAndAdd(new AutoOuttakeArmAxonAction(0.4))
                .stopAndAdd(new AutoOuttakeSliderAction(0, 1));

        TrajectoryActionBuilder collectSpec2 = scoreSpec1.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(40, -57), Math.toRadians(-90), velFast, accFast);
        //  .strafeToSplineHeading(new Vector2d(40, -60), Math.toRadians(-90));

        TrajectoryActionBuilder scoreSpec2 = collectSpec2.endTrajectory().fresh()
                .setReversed(true)
                .strafeToSplineHeading(new Vector2d(3, -40), Math.toRadians(-90))
                .waitSeconds(0.1)
                .lineToY(-33);

        TrajectoryActionBuilder collectSpec3 = scoreSpec2.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(40, -57), Math.toRadians(-90));
        //  .strafeToSplineHeading(new Vector2d(40, -60), Math.toRadians(-90));

        TrajectoryActionBuilder scoreSpec3 = collectSpec3.endTrajectory().fresh()
                .setReversed(true)
                .strafeToSplineHeading(new Vector2d(0, -40), Math.toRadians(-90))
                .waitSeconds(0.1)
                .lineToY(-33);





        //*****LOOP wait for start, INIT LOOP***********************************************************
        while (!isStarted() && !isStopRequested()) {
            //         RRAutoCoreInitLoop();
            /**PLACE Huskylens code here for testing purpose*/

            /**then PLACE LED reading here for reading purpose*/

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
                        //Initialize
                        //autoOuttakeArmAxonAction(0.9),        //rotate outtake arm to scoring position
                        //autoOuttakeSliderAction(1700,1),      //raise outtake  slider to scoring position
                        preloadScore.build(),

                        //autoClawAction(0.7),                  //open claw

                        preloadMoveBack.build(),
                        gotoSample4.build(),
                        pushSample4HomeThenToSample5.build()



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

    /**positions of OuttakeSlider: 0=ground, max/high basket=3400; high specimen bar=1300  */
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


    /**safe range for OuttakeArmAxon = 0.29 to 0.35 to 0.37 to 0.68 to 0.86 */
    public class AutoOuttakeArmAxonAction implements Action {
        private boolean initialized = false;
        double desirePosition;
        ElapsedTime timer;

        public   AutoOuttakeArmAxonAction(double position) {
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
    public Action autoOuttakeArmAxonAction(double position){
        return new AutoOuttakeArmAxonAction(position);
    }


    /**safe range for  outtakeExtension =                                */
    public class AutoouttakeExtensionAction implements Action {
        private boolean initialized = false;
        double desirePosition;
        ElapsedTime timer;

        public   AutoouttakeExtensionAction(double position) {
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
    public Action autoouttakeExtensionAction(double position){
        return new AutoouttakeExtensionAction(position);
    }


    /**safe range for cLAW =                                */
    public class AutoClawAction implements Action {
        private boolean initialized = false;
        double desirePosition;
        ElapsedTime timer;

        public   AutoClawAction(double position) {
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
    public Action autoClawAction(double position){

        return new AutoClawAction(position);
    }


    /**safe range for IntakeSlider MOTOR                               */
    public class AutoIntakeSliderAction implements Action {
        private boolean initialized = false;
        int desirePosition;
        double desiredPower;
        ElapsedTime timer;
        public   AutoIntakeSliderAction(int position, double power) {
            this.desirePosition = position;this.desiredPower = power;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Intake.intakeSlideSetPositionPower(desirePosition,desiredPower);
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
    public Action autoIntakeSliderAction(int position, double power){
        return new AutoIntakeSliderAction(position, power);
    }


    /**safe range for intakeServoAxon =                                */
    public class AutoIntakeServoAxonAction implements Action {
        private boolean initialized = false;
        double desirePosition;
        ElapsedTime timer;

        public   AutoIntakeServoAxonAction(double position) {
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
    public Action autoIntakeServoAxonAction(double position){
        return new AutoIntakeServoAxonAction(position);
    }


    public class AutoIntakeSpinerAction implements Action {
        private boolean initialized = false;
        double power;
        ElapsedTime timer;
        public   AutoIntakeSpinerAction(double power) {

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
    public Action autoIntakeSpinerAction(double power){
        return new AutoIntakeSpinerAction(power);
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