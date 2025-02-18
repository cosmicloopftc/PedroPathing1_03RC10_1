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

/** 1/5/2025:   Update to 1.0.5 - Added a MotorDirections opmode in the
 *              quickstart to test the direction of each motor.
 *
 */

public class FConstants {
    static {
        //FollowerConstants.localizers = Localizers.THREE_WHEEL;        //TODO

        FollowerConstants.localizers = Localizers.PINPOINT;             //TODO: 2/18/2025 for pinpoint
        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftRear";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightRear";

        //default is DcMotorSimple.Direction....
        FollowerConstants.leftFrontMotorDirection = DcMotorEx.Direction.REVERSE;    // default  REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorEx.Direction.REVERSE;     //update for chain rear wheel 1/6/2025 //FORWARD;     //TODO
        FollowerConstants.rightFrontMotorDirection = DcMotorEx.Direction.FORWARD;   //TODO
        FollowerConstants.rightRearMotorDirection = DcMotorEx.Direction.FORWARD;    //update for chain rear wheel 1/6/2025 REVERSE;    //default  FORWARD;

        FollowerConstants.mass = 13.3;    //V2 robot 2/18/2025    // default 13;

        //TODO: need to adjust given the new Strafe Odometry pod
        FollowerConstants.xMovement = 60;       //pinpoint;  3-wheel=  62.5387;  //55.5824; // pre-1.0.4: 60.2411;   // default 57.8741;     //v1=60; section 4.1 ForwardVelocityTuner 12/15/2024, battery V=13.6
        FollowerConstants.yMovement = 41.5;     //pinpoint;  3-wheel= 45.2512;  //38.2663; // pre-1.0.4: 43.7032;   // default 52.295;      //V1=46; section 4.2 Strafe VelocityTuner 12/15/2024, battery V=13.6

        //TODO: need to adjust given the new Strafe Odometry pod
        FollowerConstants.forwardZeroPowerAcceleration = -31;       //pinpoint;  3-wheel=  -31.9264; //-51.1683; // pre-1.0.4: -34.8325;   //-140 default  -41.278;   //V1=-37; section 5.1 forwardZeroPowerAccelerationTuner 12/15/2024, battery V=13.5
        FollowerConstants.lateralZeroPowerAcceleration = -69;       //pinpoint;  3-wheel=  -77.7008; //-75.6232; // ore-1.0.4: -74.4311;   // default-59.7819;  //V1=-65 (variable -63 to -68)  section 5.2 lateralZeroPowerAccelerationTuner 12/15/2024, battery V=13.5

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0);          //<--2/16/2025 for Pedro1.0.8    = new CustomPIDFCoefficients(0.1,0,0.01,0); //TODO //V1=0.41,0,0.050,0;
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0);  //<--2/16/2025 for Pedro1.0.8   = new CustomPIDFCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID  //TODO //V1=0.2,0,0.02,0;

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.1,0);          //<--2/16/2025 for Pedro1.0.8     = new CustomPIDFCoefficients(2,0,0.1,0);   //TODO //V1=2,0,0.02,0
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); //<--2/16/2025 for Pedro1.0.8      = new CustomPIDFCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID   //TODO //V1=4,0,0.01,0

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.06,0,0.0015,0.6,0);   //before pinpoint (0.03,0,0.001,0.6,0);          //<--2/16/2025 for Pedro1.0.8      = new CustomFilteredPIDFCoefficients(0.1,0,0,0.6,0);    //TODO //V1=0.01,0,0.000001,0.6,0
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.008,0,0.0001,0.6,0.1); //<--2/16/2025 for Pedro1.0.8      = new CustomFilteredPIDFCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID  //TODO //V1=0.009,0.000005,0.6,0

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;      //3   default 4;      //TODO //V1=4     //section 9. Drive PID
        FollowerConstants.centripetalScaling = 0.0005;              //TODO //V1=0.000020

        FollowerConstants.pathEndTimeoutConstraint = 500;           //V1=500
        FollowerConstants.pathEndTValueConstraint = 0.995;          //V1=0.995
        FollowerConstants.pathEndVelocityConstraint = 0.1;          //V1=0.1
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;         //V1=0.007




    }
}
