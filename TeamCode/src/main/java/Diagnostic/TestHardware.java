package Diagnostic;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import Hardware.HardwareDrivetrain;
import Hardware.HardwareNoDriveTrainRobot;
import Hardware.HardwareTestDevice;

import java.util.ArrayList;

// THIS CLASS SET UP THE HARDWAREMAP AND VARIOUS HARDWARE TEST CHOICES.  THIS IS THEN PULL INTO CLASS TestWIRING_OpMODE


@Disabled
public class TestHardware {
    public IMU imu;

    HardwareMap hardwareMap = null;
    HardwareNoDriveTrainRobot hardwareNoDriveTrainRobot = new HardwareNoDriveTrainRobot();
    HardwareDrivetrain hardwareDrivetrainRobot = new HardwareDrivetrain();
    HardwareTestDevice hardwareTestDevice = new HardwareTestDevice();

    private double leftFront_ticksPerRotation;

    private double testMotor_ticksPerRotation;
    private double leftRear_ticksPerRotation;
    private double rightFront_ticksPerRotation;
    private double rightRear_ticksPerRotation;
    private AngleUnit angleUnit;

    public void init(HardwareMap hardwareMap) {
        hardwareDrivetrainRobot .init(hardwareMap);
        hardwareNoDriveTrainRobot.init(hardwareMap);
        hardwareTestDevice.init(hardwareMap);

        testMotor_ticksPerRotation = hardwareTestDevice.motorTest.getMotorType().getTicksPerRev();

        leftFront_ticksPerRotation = hardwareDrivetrainRobot.leftFront.getMotorType().getTicksPerRev();
        leftRear_ticksPerRotation = hardwareDrivetrainRobot.leftRear.getMotorType().getTicksPerRev();
        rightFront_ticksPerRotation = hardwareDrivetrainRobot.rightFront.getMotorType().getTicksPerRev();
        rightRear_ticksPerRotation = hardwareDrivetrainRobot.rightRear.getMotorType().getTicksPerRev();

        imu = hardwareMap.get(IMU.class, "imu");
        //Define how the hub is mounted on robot to get correct Yaw, Pitch and Roll values.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        //Initialize IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

    }

    //********* METHODS ****
    public void motorTest_setMotorSpeed(double speed) {hardwareTestDevice.motorTest.setPower(speed);}
    public double motorTest_getMotorRotations() {return hardwareTestDevice.motorTest.getCurrentPosition();}

    public void leftFront_setMotorSpeed(double speed) {hardwareDrivetrainRobot.leftFront.setPower(speed);}
    public double leftFront_getMotorRotations() {return hardwareDrivetrainRobot.leftFront.getCurrentPosition() / leftFront_ticksPerRotation;}

    public void rightFront_setMotorSpeed(double speed) {hardwareDrivetrainRobot.rightFront.setPower(speed);}
    public double rightFront_getMotorRotations() {return hardwareDrivetrainRobot.rightFront.getCurrentPosition() / rightFront_ticksPerRotation;}

    public void leftRear_setMotorSpeed(double speed) {hardwareDrivetrainRobot.leftRear.setPower(speed);}
    public double leftRear_getMotorRotations() {return hardwareDrivetrainRobot.leftRear.getCurrentPosition() / leftRear_ticksPerRotation;}

    public void rightRear_setMotorSpeed(double speed) {hardwareDrivetrainRobot.rightRear.setPower(speed);}
    public double rightRear_getMotorRotations() {return hardwareDrivetrainRobot.rightRear.getCurrentPosition() / rightRear_ticksPerRotation;}

//    public void rightOuttakeArm_setServoPosition(double position) {
//        HardwareOuttake.
//                rightOuttakeArm.setPosition(position);
//    }

    public double getHeading(AngleUnit angleUnit) {return hardwareNoDriveTrainRobot.imu.getRobotYawPitchRollAngles().getPitch(angleUnit);}

    public void setServoTestPosition( double position) {
        hardwareTestDevice.servoTest.setPosition(position);
    }

//    public double sensorDistance_getDistance(DistanceUnit du) {
//        return Sensor.sensorColorDistance.getDistance(du);
//    }

//    public int getAmountRed() {
//        return Sensor.sensorColor.red();
//    }

//    public boolean isTouchSensorPressed() {
//        return !touchSensor.getState();
//    }

//    public double getPotAngle() {
//        return Range.scale(pot.getVoltage(), 0, pot.getMaxVoltage(), 0, 270);
//    }



    public ArrayList<TestItem> getTests() {
        ArrayList<TestItem> tests = new ArrayList<>();
        tests.add(new Test_Motor("1 rightFront-positive power", 0.1, hardwareDrivetrainRobot.rightFront));
        tests.add(new Test_Motor("2 leftFront-positive power", 0.1, hardwareDrivetrainRobot.leftFront));
        tests.add(new Test_Motor("3 rightRear-positive power", 0.1, hardwareDrivetrainRobot.rightRear));
        tests.add(new Test_Motor("4 leftRear-positive power", 0.1, hardwareDrivetrainRobot.leftRear));
        tests.add(new Test_Motor("5 motorTest-positive power", 0.1, hardwareTestDevice.motorTest));

        tests.add(new Test_Servo("6 servoTest: off position=0.0, A/B=1.0", hardwareTestDevice.servoTest, 0.0, 1.0));

        //tests.add(new TestDigitalChannel_14_1("PB Touch", touchSensor));

        tests.add(new Test_IMU("7 IMU", imu));

        return tests;
    }
}