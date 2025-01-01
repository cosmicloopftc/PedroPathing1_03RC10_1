package Hardware;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
//import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
//import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
//import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
//import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure

public class HardwareDrivetrain {
    public static DcMotorEx leftFront = null;
    public static DcMotorEx leftRear = null;
    public static DcMotorEx rightFront = null;
    public static DcMotorEx rightRear = null;


    double newForward = 0, newRight = 0, driveTheta = 0, r = 0 ;
    /*Constructor*/
    public HardwareDrivetrain() {
    }


    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap)    {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
//        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        setMotorPower(0,0,0,0);
    }


    public void start(){

    }


    public void stop (){
        setMotorPower(0,0,0,0);
    }

    public static void setMotorPower(double RF, double LF, double RB, double LB) {
        rightFront.setPower(RF);
        leftFront.setPower(LF);
        rightRear.setPower(RB);
        leftRear.setPower(LB);
    }

    //12/18/2024 BELOW IS COMMENTED OUT SINCE THERE IS NO NEED FOR DRIVETRAIN METHOD IN THIS VERSION
//    simplify driving move, based on Learn Java for FTC p. 145-146, Oct 2023
//    botheading is used for fieldOriented (driver's perspective driving)
    public void drive(double forward, double right, double rotate, double power, double botHeading, String drivingOrientation) {
        if (drivingOrientation == "fieldOriented") {
            driveTheta = Math.atan2(forward, right);                         //convert to polar
            r = Math.hypot(forward, right);                                  //convert to polar
            driveTheta = AngleUnit.normalizeRadians(driveTheta - botHeading);       // rotate angle
            newForward = r * Math.sin(driveTheta);
            newRight = 1.1 * r * Math.cos(driveTheta);  //factor = 1.1; correct for strafe imperfection; convert back to cartesian
        }
        if (drivingOrientation == "robotOriented") {
            newForward = forward;
            newRight = right;
        }
        double denominator = Math.max(Math.abs(newForward) + Math.abs(newRight) + Math.abs(rotate), 1);
        double frontLeftPower =     power*(newForward + newRight + rotate)/denominator;
        double frontRightPower = power*(newForward - newRight - rotate)/denominator;
        double backLeftPower = power*(newForward - newRight + rotate)/denominator;
        double backRightPower = power*(newForward + newRight - rotate)/denominator;

        setMotorPower(frontRightPower, frontLeftPower, backRightPower, backLeftPower);
    }

}