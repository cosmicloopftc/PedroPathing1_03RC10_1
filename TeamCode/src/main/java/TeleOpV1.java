
import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.DashboardPoseTracker;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Hardware.HardwareDrivetrain;
import Hardware.HardwareIntake;
import Hardware.HardwareNoDriveTrainRobot;

//import pedroPathing.follower.Follower;
//import pedroPathing.localization.Pose;
//import pedroPathing.localization.PoseUpdater;
//import pedroPathing.util.DashboardPoseTracker;

import android.provider.SyncStateContract;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is the TeleOpEnhancements OpMode. It is an example usage of the TeleOp enhancements that
 * Pedro Pathing is capable of.Anyi Lin, Aaron Yang, Harrison Womack - 10158 Scott's Bots, @version 1.0, 3/21/2024

 8/26/2024:  update motor map in this class, in PedroPathing/follower/Follower, in PedroPathing/localization/localizers/ThreeWheelLocalizer
 9/2/2024WT: add FTC dashboard lines similar to those in LocalizerTest
 10/13/2024OT: transfer over PedroPath TeleOp from previous Pedropath tuning
 10/13/2024WT: transfer PedroPath tuning info
 10/14/2024MT/WT: add manual driving, not using PedroPath Follower
 10/24/2024WT: correct HardwareLED class, rename in AdafruitLED object name--12/16/2024---LED HardwareLED() with both regualr LED and Adafruit long LED string
 11/1/2024: Allow for high and low basket scoring with no intaking
 11/23/2024: Allow for more than 1 time intaking + wall intake in start state w/specimen positions
 11/26/2024: Fix the outtake finite state machine conditions
 11/27/2024: Fixed and improved specimen scoring
 11/28/2024: Allow to go from specimen scoring directly back to wall intake
 11/30/2024, 12/17/2024:  Add TelemetryA and Drawing for FTC Dashboard access
 12/17/2024:    add slide motor current draw
 12/18/2024: note HardwareNoDriveTrainRobot = pull all hardware mapping/function into here except for drivetrain motors
 HardwareRobot = pull all hardware mapping/function including drivetrain motors
 Add PedroPathing teleop control (Note: Follower object has the drivetrain motor mapping already),
 Fix option for non-PedroPathing teleop control if desired later
 by using HardwareDrivetrain and HardwareNoDrivetrainRobot separately
 Activate PedroPathing teleop by: 1) use follower.startTeleopDrive() in start() loop and comment out drive comments in loop()
 Use regular driving with the opposite of above.
 12/27/2024: add Color sensor for color, hue, distance based on FIRST external example
 1/1/2025:   Update Diagnostic
 1/3/2025:   Update new PedroPath 1.0.3 teleOp movement
                TODO: add AUTOconstant of data transfer from end of AUTO
 */



@Config    //need this to allow appearance in FtcDashboard Configuration to make adjust of variables
@TeleOp(group="Primary", name= "TeleOpV1.2")
public class TeleOpV1 extends OpMode {
    private Telemetry telemetryA;

    //based on Robot-Centric Teleop  from @author Baron Henderson - 20077 The Indubitables
    // * @version 2.0, 11/28/2024
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);  //TODO: Later, reset this to transfer location from Auto


    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    public static double intakeSlidesCurrent;

    //label robot for all hardware except drivetrain; and drivetrain is a separate object
    HardwareNoDriveTrainRobot robot = new HardwareNoDriveTrainRobot();
    HardwareDrivetrain drivetrain = new HardwareDrivetrain();

    VoltageSensor battery;


    enum State{
        START,
        INTAKE,
        TRANSFER,
        OUTTAKE_READY,
        OUTTAKE,
        READY_DOWN
    }
    State state = State.START;



    private ElapsedTime runtime = new ElapsedTime();
    double botHeading;
    String drivingOrientation = "robotOriented";   //TODO: as default for Eduardo, but will also reset in init as well.
    double lastTime;
    double imuAngle;
    String outtakeOption = "";


    //Declare variables for standard driving--not using PedroPath follower method (using Learn JAVA for FTC book)
    double y, x, rx, powerShift;
    //double newForward = 0, newRight = 0, driveTheta = 0, r = 0, powerShift = 0;


    //Declare variables for Color sensor for color, hue, distance
    float colorGain = 2;
    // Once per loop, we will update this hsvValues array.
    // first element (0)= hue, second element (1)=saturation, third element (2)= value.
    // See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
    // for an explanation of HSV color.
    final float[] hsvValues = new float[3];
    // xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of the previous and current
    // state of the X button on the gamepad
    boolean xButtonPreviouslyPressed = false;
    boolean xButtonCurrentlyPressed = false;


    //__________________________________________________________________________________________________
    @Override
    public void init() {
        poseUpdater = new PoseUpdater(hardwareMap);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


        robot.init(hardwareMap);   //for all hardware except drivetrain.  note hardwareMap is default and part of FTC Robot Controller HardwareMap class
        robot.imu.resetYaw();      //reset the IMU/Gyro angle with each match.
        drivetrain.init(hardwareMap);   //for drivetrain only

        battery = hardwareMap.voltageSensor.get("Control Hub");


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


//        follower.startTeleopDrive();
//        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
//        Drawing.sendPacket();



        telemetryA.addData(">", "Hardware Initialization complete");
        telemetryA.update();
        runtime.reset();

    }


    @Override
    public void init_loop() {
        //telemetryA.addData("Present Heading by IMU in degree = ", "(%.1f)", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        bulkReadTELEOP();
        botHeading = imuAngle;

//        robot.LED.LEDinitReady();
//
//        telemetryA.setMsTransmissionInterval(50);
//        telemetryA.addData("Battery Voltage (V): ", "%.1f", battery.getVoltage());
//        telemetryA.addData("Bot Heading--imu Yaw (degrees): ", "%.1f", botHeading);
//        telemetryA.addData("Robot Driving Orientation if not PedroPath = ", drivingOrientation);
//        telemetryA.addLine("");
//        telemetryA.addData("Outtake left slide position: ", robot.Outtake.outtakeRightSlide.getCurrentPosition());
//        telemetryA.addData("Outtake right slide position: ",  robot.Outtake.outtakeLeftSlide.getCurrentPosition());
//        telemetryA.addLine("");
//        telemetryA.addLine("current check--current spike if stalling");
//        //when stalling/spike is detected, it means the slide is at lowest or max position and so can reset it as zero position
//        telemetryA.addData("Outtake left slide motor current (mA): ", "%.1f", robot.Outtake.outtakeRightSlide.getCurrent(CurrentUnit.MILLIAMPS));
//        telemetryA.addData("Outtake right slide motor current (mA): ",  "%.1f", robot.Outtake.outtakeLeftSlide.getCurrent(CurrentUnit.MILLIAMPS));
//
//
//        telemetryA.addData("red", robot.Sensor.colorTest.red());
//        telemetryA.addData("hue", robot.Sensor.getColorInfo(0));
//        telemetryA.addLine();

        telemetryA.addData("Runtime (seconds) = ", "%.1f", getRuntime());
        telemetryA.addData("Distance Sensor = ", "%.1f", robot.Sensor.getDistance());
        telemetryAllColorInfo();
        gamePadColorControl();




        telemetryA.update();
    }

    //-------------------------------------------------------------------------------------------------
    @Override
    public void start() {
        robot.start();
        runtime.reset();
        drivingOrientation = "robotOriented";
        state = State.START;

        //This starts teleop drive control by 1. breakFollowing() and set teleopDrive = true;
        //If regular manual control by JAVA for FTC method, then comment this out.
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        bulkReadTELEOP();
        botHeading = imuAngle;





        switch (state) {
            case START:
                if(gamepad2.a || outtakeOption.equals("start")) {
                    robot.Outtake.groundPositionOpen();
                    outtakeOption = "";
                }
                if(gamepad2.dpad_down){
                    outtakeOption = "wallIntake";
                    state = State.OUTTAKE_READY;
                }
                if(gamepad1.dpad_up){
                    robot.Intake.intakeSlideOUT();
                    robot.Intake.intakeDOWN();
                    //robot.Intake.intakeIN();
                }
                else if(gamepad1.dpad_left){
                    robot.Intake.intakeSlideMID();
                    robot.Intake.intakeDOWN();
                    //robot.Intake.intakeIN();
                }
                else if(gamepad1.dpad_right){
                    robot.Intake.intakeSlideIN();
                    robot.Intake.intakeDOWN();
                    //robot.Intake.intakeIN();
                }
                else if(gamepad1.dpad_down){
                    robot.Intake.intakeSTOP();
                    robot.Intake.intakeUP();
                    robot.Intake.intakeSlideIN();
                }
                if (gamepad1.left_trigger > 0.2){
                    robot.Intake.intakeIN();
                }
                else if (gamepad1.left_bumper){
                    robot.Intake.intakeOUT();
                }
                if (robot.Intake.intakeSlides.getCurrentPosition() < 10 && robot.Intake.leftIntakeServo.getPosition() == 1 && gamepad1.right_trigger > 0.2) {
                    robot.Intake.intakeOUT();
                    state = State.TRANSFER;
                }
                break;
            case INTAKE:
                break;
            case TRANSFER:
                if (gamepad1.right_trigger > 0.2) {
                    robot.Intake.intakeOUT();
                }
                else if (gamepad1.left_trigger > 0.2){
                    robot.Intake.intakeSTOP();
                }
                else if (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right){ //Be able to intake again
                    state = State.START;
                }
                if(gamepad2.a || outtakeOption.equals("start")) { // this current code would be in transfer state when intake ready
                    robot.Outtake.groundPositionOpen();
                    outtakeOption = "";
                }
                else if(gamepad2.left_trigger > 0.2 || gamepad2.right_trigger > 0.2){
                    robot.Outtake.groundPositionClose();
                }
//                if(gamepad2.b) { // would having these in this else if statement make sure that the claw is closed?
//                    outtakeOption = "lowBasket";
//                    //robot.Outtake.closeClaw();
//                    state = State.OUTTAKE_READY;
//                }
                if(robot.Outtake.claw.getPosition() == 0.32 && gamepad2.y) { //Make sure that claw is in closed position
                    outtakeOption = "highBasket";
                    //robot.Outtake.closeClaw();
                    state = State.OUTTAKE_READY;
                }
//                if(gamepad2.b) {
//                    outtakeOption = "lowBasket";
//                    robot.Outtake.closeClaw();
//                    state = State.OUTTAKE_READY;
//                }
//                else if(gamepad2.y) {
//                    outtakeOption = "highBasket";
//                    robot.Outtake.closeClaw();
//                    state = State.OUTTAKE_READY;
//                }
                else if(gamepad2.dpad_down){
                    outtakeOption = "wallIntake";
                    state = State.OUTTAKE_READY;
                }
                break;
            case OUTTAKE_READY:
                robot.Outtake.readyPosition();
                if(outtakeOption.equals("highBasket") && robot.Outtake.outtakeLeftSlide.getCurrentPosition()>0){
                    state = State.OUTTAKE;
                }
                if(outtakeOption.equals("wallIntake") && robot.Outtake.outtakeLeftSlide.getCurrentPosition()>400){
                    state = State.OUTTAKE;
                }
                break;

            case OUTTAKE:
//                if (outtakeOption.equals("lowBasket")){
//                    robot.Outtake.lowBasket();
//                    if (gamepad2.left_bumper || gamepad2.right_bumper){
//                        robot.Outtake.openClaw();
//                    }
//                    if (gamepad2.a){ // Move slides down
//                        resetRuntime();
//                        robot.Outtake.leftOuttakeArm.setPosition(0.98);
//                        robot.Outtake.rightOuttakeArm.setPosition(0.02);
//                        if (getRuntime() > 0.1){
//                            state = State.READY_DOWN;
//                        }
//                    }
//                }
                if (outtakeOption.equals("highBasket")){
                    robot.Outtake.leftSlideSetPositionPower(3400,1);
                    robot.Outtake.rightSlideSetPositionPower(3400,1);
                    if (robot.Outtake.outtakeLeftSlide.getCurrentPosition()>1500){
                        robot.Outtake.highBasket();
                    }
                }
                if (robot.Outtake.outtakeLeftSlide.getCurrentPosition() > 3200 && gamepad2.left_bumper || gamepad2.right_bumper){ // If at high basket position
                    robot.Outtake.openClaw();
                }
                else if (robot.Outtake.outtakeLeftSlide.getCurrentPosition() > 3200 && gamepad2.a){ // Should robot make sure claw is open before going down
                    state = State.READY_DOWN;
                }

                if (outtakeOption.equals("wallIntake")){
                    robot.Outtake.openClaw();
                    robot.Outtake.wallIntake();
                }
//                if (robot.Outtake.leftOuttakeArm.getPosition() == 0.05 && gamepad2.left_trigger > 0.2 || gamepad2.right_trigger > 0.2){ // Only if at wall intake position
//                    robot.Outtake.closeClaw();
//                    outtakeOption = "highChamber";
//                }
//
//                if (outtakeOption.equals("highChamber")){
//                    robot.Outtake.highChamberSet();
//                }
//                if (robot.Outtake.leftOuttakeArm.getPosition() == 0.3 && gamepad2.left_bumper || gamepad2.right_bumper){ // Only if at high chamber set position
//                    outtakeOption = "highChamberFinish";
//                }
//
//                if (outtakeOption.equals("highChamberFinish")){
//                    robot.Outtake.highChamberFinish();
//                    //robot.Outtake.openClaw();
//                }
//                if (robot.Outtake.outtakeLeftSlide.getCurrentPosition() > 800 && robot.Outtake.outtakeLeftSlide.getCurrentPosition() < 1000 && gamepad2.left_bumper || gamepad2.right_bumper){ // If at high chamber finish position
//                    robot.Outtake.openClaw();
//                }

                if (robot.Outtake.outtakeLeftSlide.getCurrentPosition() > 600 && robot.Outtake.outtakeLeftSlide.getCurrentPosition() < 800 && robot.Outtake.claw.getPosition() == 0.17 && gamepad2.a){ // Only if at high Chamber finish position and claw is open
                    state = State.READY_DOWN;
                }
                else if (robot.Outtake.outtakeLeftSlide.getCurrentPosition() > 600 && robot.Outtake.outtakeLeftSlide.getCurrentPosition() < 800 && robot.Outtake.claw.getPosition() == 0.17 && gamepad2.dpad_down){ // Only if at high Chamber finish position and claw is open
                    outtakeOption = "wallIntake";
                }

//                if (gamepad2.a){ // Move slides down
//                    if (outtakeOption.equals("lowBasket")){ //Makes sure arm does not hit intake when going down
//                        resetRuntime();
//                        robot.Outtake.leftOuttakeArm.setPosition(0.98);
//                        robot.Outtake.rightOuttakeArm.setPosition(0.02);
//                        if (getRuntime() > 0.1){
//                            state = State.READY_DOWN;
//                        }
//                    }
//                    else{
//                        state = State.READY_DOWN;
//                    }
//                }
                break;
            case READY_DOWN:
                robot.Outtake.readyPosition();
                if(robot.Outtake.outtakeLeftSlide.getCurrentPosition()<600){
                    outtakeOption = "start";
                    state = State.START;
                }
                break;
        }


        //Hang testing:
        // Move slides up before hanging
        if (gamepad2.dpad_left) {
            robot.Outtake.leftSlideSetPositionPower(3400, 1);
            robot.Outtake.rightSlideSetPositionPower(3400, 1);
        }
        // Pull slides down to hang
        else if(gamepad2.dpad_right) {
            robot.Outtake.leftSlideSetPositionPower(1500, 1);
            robot.Outtake.rightSlideSetPositionPower(1500, 1);
        }

//Drivetrain Movement:
//MANUAL DRIVE for Mecanum wheel drive.
        y = -gamepad1.left_stick_y;           // Remember,joystick value is reversed!
        rx = gamepad1.left_stick_x;             // this is strafing  V1=positive
        x = gamepad1.right_stick_x;                // this is strafing  V1=positive

        //Cancel angle movement of gamepad left stick, make move move either up/down or right/left
        //if (Math.abs(y) >= Math.abs(x)) {
        //  y = y;
        //  x = 0;
        //  } else {
        //      y = 0;
        //      x = x;
        //  }
        //DRIVETRAIN
        //baseline speed =  reduce motor speed to 60% max
        double motorPowerDefault = 0.6;
        double powerChange;

        //SLOW DOWN with RIGHT LOWER TRIGGER (lower of the top side button) press with the other gamepad stick.
        if ((Math.abs(gamepad1.left_stick_y) > 0.1 && gamepad1.right_trigger > 0.1) || (Math.abs(gamepad1.left_stick_x) > 0.1 && gamepad1.right_trigger > 0.1) || (Math.abs(gamepad1.right_stick_x) > 0.1 && gamepad1.right_trigger > 0.1)) {
            powerChange = -0.3;
            //SPEED UP with RIGHT UPPER BUMPER (up of the top side button) press with the other gamepad stick.
        } else if ((Math.abs(gamepad1.left_stick_y) > 0.1 && gamepad1.right_bumper) || (Math.abs(gamepad1.left_stick_x) > 0.1 && gamepad1.right_bumper) || (Math.abs(gamepad1.right_stick_x) > 0.1 && gamepad1.right_bumper)) {
            powerChange = 0.4;
        } else {
            powerChange = 0;
        }
        powerShift = motorPowerDefault + powerChange;

/**12/18/2024--THIS IS COMMENTED OUT WHEN USING PEDROPATHING TO DRIVE ROBOT
 drivetrain.drive(y, x, rx, powerShift, botHeading, drivingOrientation);
 */
/** Comment out if using regular drivetrain to drive robot, otherwise use below.
 * Update Pedro to move the robot based on:
 - Forward/Backward Movement: -gamepad1.left_stick_y
 - Left/Right Movement: -gamepad1.left_stick_x
 - Turn Left/Right Movement: -gamepad1.right_stick_x
 - Robot-Centric Mode: true
 --original:  follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
 */
        //follower.setTeleOpMovementVectors(-y*powerShift, -x*powerShift, -rx*powerShift, true);
        //Below set the max power as set above
        follower.setTeleOpMovementVectors(
                Range.clip(y,-powerShift, +powerShift),
                Range.clip(x,-powerShift, +powerShift),
                Range.clip(rx,-powerShift, +powerShift), true);
        follower.update();



        telemetryA.addData("X ", follower.getPose().getX());
        telemetryA.addData("Y ", follower.getPose().getY());
        telemetryA.addData("Bot Heading--PedroPathing (degrees)", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetryA.addLine("");
        telemetryA.addData("Bot Heading--imu Yaw (degrees)", "%.1f", botHeading);
        telemetryA.addLine("");
        telemetryA.addData("Runtime (seconds) = ", "%.1f", getRuntime());
        //telemetryA.addData("Robot Driving Orientation = ", drivingOrientation);
        telemetryA.addData("State = ", state);
        telemetryA.addData("Time in State (seconds) = ", 0);
        telemetryA.addData("lastTime = ", lastTime);

        telemetryA.addData("OuttakeSliderLeftCurrent mA = ", robot.Outtake.getOuttakeSliderLeftCurrent());
        telemetryA.addData("OuttakeSliderRightCurrent mA = ", robot.Outtake.getOuttakeSliderRightCurrent());


//        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
//        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
//        Drawing.sendPacket();
//        telemetry.update();
        telemetryA.update();


    }


    @Override
    public void stop() {
        robot.stop();
        drivetrain.setMotorPower(0, 0, 0, 0);
    }


    public void bulkReadTELEOP() {
        imuAngle = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);       //0. direction is reverse
        //newRightEncoder = robot.RBack_Motor.getCurrentPosition();       //1.
    }

    public void telemetryAllColorInfo(){
        telemetryA.addData("Gain", colorGain);
        telemetryA.addLine("");
        // Tell sensor desired gain value (normally you would do this during initialization, not during loop)
        robot.Sensor.colorTest.setGain(colorGain);
        // Get the normalized colors from the sensor
        NormalizedRGBA colors = robot.Sensor.colorTest.getNormalizedColors();

        /* Use telemetry to display feedback on Driver Station. Show red/green/blue normalized values
         *from sensor (0 to 1), and equivalent HSV (hue/saturation/value) values.
         * See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for explanation of HSV color. */
        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);
        telemetryA.addData("Red", "%.3f", colors.red);
        telemetryA.addData("Green", "%.3f", colors.green);
        telemetryA.addData("Blue", "%.3f", colors.blue);
        telemetryA.addLine("");
        telemetryA.addData("Hue", "%.3f", hsvValues[0]);
        telemetryA.addData("Saturation", "%.3f", hsvValues[1]);
        telemetryA.addData("Value", "%.3f", hsvValues[2]);
        telemetryA.addLine("");
        telemetryA.addData("Alpha", "%.3f", colors.alpha);
        telemetryA.addLine("");
        /* If this color sensor also has a distance sensor, display the measured distance.
         * Note that the reported distance is only useful at very close range, and is impacted by
         * ambient light and surface reflectivity. */
        if (robot.Sensor.colorTest instanceof DistanceSensor) {
            telemetryA.addData("Distance (cm)", "%.3f", ((DistanceSensor) robot.Sensor.colorTest).getDistance(DistanceUnit.CM));
        }
    }

    public void gamePadColorControl(){
        telemetryA.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
        telemetryA.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

        // Update the gain value if either of the A or B gamepad buttons is being held
        if (gamepad1.a) {
            // Only increase the gain by a small amount, since this loop will occur multiple times per second.
            colorGain += 0.005;
        } else if (gamepad1.b && colorGain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
            colorGain -= 0.005;
        }
        // Check the status of the X button on the gamepad
        xButtonCurrentlyPressed = gamepad1.x;

        // If the button state is different than what it was, then act
        if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
            // If the button is (now) down, then toggle the light
            if (xButtonCurrentlyPressed) {
                if (robot.Sensor.colorTest instanceof SwitchableLight) {
                    SwitchableLight light = (SwitchableLight)robot.Sensor.colorTest;
                    light.enableLight(!light.isLightOn());
                }
            }
        }
        xButtonPreviouslyPressed = xButtonCurrentlyPressed;

    }






}