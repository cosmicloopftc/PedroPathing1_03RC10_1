package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.THREE_WHEEL;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftRear";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightRear";

        FollowerConstants.leftFrontMotorDirection = DcMotorEx.Direction.REVERSE;    // default  REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorEx.Direction.FORWARD;     //TODO
        FollowerConstants.rightFrontMotorDirection = DcMotorEx.Direction.FORWARD;   //TODO
        FollowerConstants.rightRearMotorDirection = DcMotorEx.Direction.REVERSE;    //default  FORWARD;

        FollowerConstants.mass = 12.2;      // default 13;

        FollowerConstants.xMovement = 61;   // default 57.8741;     //v1=60; section 4.1 ForwardVelocityTuner 12/15/2024, battery V=13.6
        FollowerConstants.yMovement = 45;   // default 52.295;      //V1=46; section 4.2 Strafe VelocityTuner 12/15/2024, battery V=13.6

        FollowerConstants.forwardZeroPowerAcceleration = -36;   // default  -41.278;   //V1=-37; section 5.1 forwardZeroPowerAccelerationTuner 12/15/2024, battery V=13.5
        FollowerConstants.lateralZeroPowerAcceleration = -78;   // default-59.7819;  //V1=-65 (variable -63 to -68)  section 5.2 lateralZeroPowerAccelerationTuner 12/15/2024, battery V=13.5

        FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(0.1,0,0.01,0); //TODO //V1=0.41,0,0.050,0;
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID  //TODO //V1=0.2,0,0.02,0;

        FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(2,0,0.1,0);   //TODO //V1=2,0,0.02,0
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID   //TODO //V1=4,0,0.01,0

        FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.1,0,0,0.6,0);    //TODO //V1=0.01,0,0.000001,0.6,0
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID  //TODO //V1=0.009,0.000005,0.6,0

        FollowerConstants.zeroPowerAccelerationMultiplier = 6;      //default 4;      //TODO //V1=4     //section 9. Drive PID
        FollowerConstants.centripetalScaling = 0.0005;              //TODO //V1=0.000020

        FollowerConstants.pathEndTimeoutConstraint = 500;           //V1=500
        FollowerConstants.pathEndTValueConstraint = 0.995;          //V1=0.995
        FollowerConstants.pathEndVelocityConstraint = 0.1;          //V1=0.1
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;         //V1=0.007




    }
}
