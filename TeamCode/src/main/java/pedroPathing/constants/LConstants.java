package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
        /**THIS IS FOR 3 wheel Localizer
         *Follow instruction on GoBilda website
         */

//        ThreeWheelConstants.forwardTicksToInches = 0.00197;    //default  .001989436789;
//        ThreeWheelConstants.strafeTicksToInches = 0.00197;     //new one to match forward given all 3 same Gobilda pods 2/16/2025;  0.00295;       //default  .001989436789;
//        ThreeWheelConstants.turnTicksToInches = 0.001827;        //TODO: old was TURN_TICKS_TO_RADIANS, so need to check.  default  .001989436789;
//        ThreeWheelConstants.leftY = 5.25;       //default  1;
//        ThreeWheelConstants.rightY =-5.25;        //default  -1;
//        ThreeWheelConstants.strafeX = -1.25;       //TODO: need to adjust 2/16/2025   default  -2.5;
//        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftRear";       //default  "leftFront";
//        ThreeWheelConstants.rightEncoder_HardwareMapName = "rightFront";       //default  "rightRear";
//        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightRear";       //default  "rightFront";
//        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;    //TODO     //default  REVERSE;
//        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;    //TODO    //default  REVERSE;
//        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;   //TODO    //default  FORWARD;


        /**THIS IS FOR PINPOINT Localizer--G
         *Follow instruction on GoBilda website
         */
        PinpointConstants.forwardY = 5.25;
        PinpointConstants.strafeX = -1.25;
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.hardwareMapName = "pinpoint";
        PinpointConstants.useYawScalar = false;             //in order to use Gobilda values
        //PinpointConstants.yawScalar = 1.0;                //not use, as we are using Golbilda
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        //PinpointConstants.customEncoderResolution = 13.26291192;      //not use, as we are using Golbilda
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;     //REVERSE
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;     //FORWARD;


        /**THIS IS FOR OTOS odometry
         *If you are using the corrected OTOS class, set OTOSConstants.useCorrectedOTOSClass to true.
         *If you are using the original OTOS class, set OTOSConstants.useCorrectedOTOSClass to false.
         *Make sure that the I2C port that you are using is the same class as the one you are using,
         * determined by above.
         */
//        OTOSConstants.useCorrectedOTOSClass = false;        //set to true or false based on above instructions
//        OTOSConstants.hardwareMapName = "OTOS";
//        OTOSConstants.linearUnit = DistanceUnit.INCH;
//        OTOSConstants.angleUnit = AngleUnit.RADIANS;
//        //or OTOSConstants.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        OTOSConstants.offset = new SparkFunOTOS.Pose2D(0, 0, Math.PI / 2);
//        //Left/right is the y axis and forward/backward is the x axis,
//        // with left being positive y and forward being positive x.
//        //PI/2 radians is facing forward, and clockwise rotation is negative rotation.
//
//        OTOSConstants.linearScalar = 1.0;   //default linear unit is DistanceUnit.INCH;  section 2.5.2a run either Forward or Lateral Localizer Tuner
//        OTOSConstants.angularScalar = 1.0;  //default angle unit is AngleUnit.RADIANS.   section 2.5.2b run Turn Localizer Tuner
//            //section 2.5. Run Localization Test.  Observe the movements, moving the robot forward
//            // should make x increase and strafing left should make y increase.





    }
}




