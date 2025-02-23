
import static Hardware.HardwareDrivetrain.leftFront;
import static Hardware.HardwareDrivetrain.leftRear;
import static Hardware.HardwareDrivetrain.rightFront;
import static Hardware.HardwareDrivetrain.rightRear;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import Hardware.HardwareDrivetrain;
import Hardware.HardwareNoDriveTrainRobot;
import RR.Drawing;
import RR.MecanumDrive;

/** This case extend from BLUE alliance TeleOp program
 * 2/23/2025        TODO: Need to test
 *
 *
 * */

@Config
@TeleOp(group="Secondary", name= "DebugOpMode Diagnostic 1.1")


public class DebugOpMode1 extends OpMode {
    VoltageSensor battery;

    int debugLevel = 499;
    HardwareNoDriveTrainRobot robot = new HardwareNoDriveTrainRobot();
    HardwareDrivetrain drivetrain = new HardwareDrivetrain();
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


    @Override
    public void init() {
        robot.init(hardwareMap);   //for all hardware except drivetrain.  note hardwareMap is default and part of FTC Robot Controller HardwareMap class
        robot.imu.resetYaw();      //reset the IMU/Gyro angle with each match.
        drivetrain.init(hardwareMap);   //for drivetrain only
        battery = hardwareMap.voltageSensor.get("Control Hub");

        debugDevice(500, "leftFront", leftFront);
        debugDevice(500, "leftRear", leftRear);
        debugDevice(500, "rightRear", rightRear);
        debugDevice(500, "rightFront", rightFront);
        debugDevice(500, "Battery Voltage", battery);

//        debugDevice(500, "colorIntake", colorIntake);
//        debugDevice(500, "LED", leds);
//        debugDevice(500, "intakeSlides", intakeSlides);
//        debugDevice(500, "intakeLeftWheel", intakeLeftWheel;
//        debugDevice(500, "intakeRightWheel", intakeRightWheel);
//        debugDevice(500, "intakeServoAxon", intakeServoAxon);
//        debugDevice(500, "sweeper", sweeper);
//        debugDevice(500, "intakeServoAxon", intakeServoAxon);
//        debugDevice(500, "intakeServoAxon", intakeServoAxonPosition);
//
//        debugDevice(500, "outtakeLeftSlide", outtakeLeftSlide);
//        debugDevice(500, "outtakeRightSlide", outtakeRightSlide);
//        debugDevice(500, "outtakeArmAxon", outtakeArmAxony);
//        debugDevice(500, "outtakeExtension", outtakeExtension);
//        debugDevice(500, "claw", claw);
//        debugDevice(500, "outtakeArmAxonPosition",outtakeArmAxonPosition);

        telemetry.update();

    }

    public void init_loop(){

    }
    public void start(){

    }

    public void loop(){
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

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

        telemetry.update();
    }

    public void stop(){

    }

    private void debugDevice(int myLevel, String myName, HardwareDevice myDevice){
        String message;
        message = myName + "== name: " + myDevice.getDeviceName()
                + ", connect: " + myDevice.getConnectionInfo()
                + ", version: " + myDevice.getVersion();
        if (debugLevel > myLevel) {
            telemetry.addData("**DEBUG DEVICE**", message);
        }
        RobotLog.i("**DEBUG DEVICE** " + message);
    }


}