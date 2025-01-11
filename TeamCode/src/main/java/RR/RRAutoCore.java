package RR;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Hardware.HardwareNoDriveTrainRobot;

public class RRAutoCore extends LinearOpMode {
    int debugLevel = 499;
    Telemetry telemetryA;
    Timer pathTimer, actionTimer, opmodeTimer;
    //HuskyLens huskyLens;

    Pose2d pose;





    //TODO: setup initial position for all subsystems
    //    public static double autoEnd_SliderMotorPosition,
    //            autoEnd_Slider_ServoArmPosition;


    public void RRAutoCoreInitLoop(){
        //TODO: place in telemetry data and alliance zone info to make sure you select the right program to run
        // Share the CPU.
        sleep(20);
    }



    HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();



    //-------------------------------------------------------------------------------------------------
    @Override
    public void runOpMode() throws InterruptedException {



    }








    void RRAutoCoreTelemetryDuringteleOp(){
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        //TODO:  methods to constantly write into into our AUTOstorage class to transfer to Teleop
    }








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







}





//////-----------------------------------------------------------
//public class AutoIntakeSlider  {
//    //HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();
//    public class AutoIntakeSlideOUT implements Action {
//        private boolean initialized = false;
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!initialized) {
//                autoRobot.Intake.intakeSlideSetPositionPower(500,0.4); //TODO: find position and power  initialized = true;
//            }
//            double positionIntakeSlide = autoRobot.Intake.intakeSlides.getCurrentPosition();
//            packet.put("Intake slider position", positionIntakeSlide);
//            if (positionIntakeSlide < 500.0) {
//                return true;
//            } else {
//                return false;
//            }
//        }
//    }
//    public Action autoIntakeSlideOUT() {
//        return new RRAutoCore.AutoIntakeSlider.AutoIntakeSlideOUT();
//    }
//
//
//    public class AutoIntakeSlideIN implements Action {
//        private boolean initialized = false;
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!initialized) {
//                autoRobot.Intake.intakeSlideSetPositionPower(0,0.4); //TODO: find position and power  initialized = true;
//                initialized = true;
//            }
//            double positionIntakeSlide = autoRobot.Intake.intakeSlides.getCurrentPosition();
//            packet.put("Intake slider position", positionIntakeSlide);
//            if (positionIntakeSlide > 0.0) {
//                return true;
//            } else {
//                return false;
//            }
//        }
//    }
//    public Action autoIntakeSlideIN(){
//        return new RRAutoCore.AutoIntakeSlider.AutoIntakeSlideIN();
//    }
//}



/**EXAMPLE OF STATE MACHINE IN AUTO for LinearOP mode
        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic. You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands. We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    // Check if the drive class isn't busy.  `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished and We move on to the next state
                    // Make sure we use the async follow function

                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case TRAJECTORY_2:
                    // Check if the drive class is busy following the trajectory. Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TURN_1;
                        drive.turnAsync(turnAngle1);
                    }
                    break;
                case TRAJECTORY_3:
                    // Check if the drive class is busy following the trajectory.  If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;
                        // Start the wait timer once we switch to the next state.  This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_1:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.TURN_2;
                        drive.turnAsync(turnAngle2);
                    }
                    break;
                case TURN_2:
                    // Check if the drive class is busy turning.  If not, move onto the next state, IDLE.  We are done with the program
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE.  currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            lift.update();
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();
            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;
            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

**/