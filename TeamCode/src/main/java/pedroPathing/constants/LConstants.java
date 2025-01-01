package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.00195;       //default  .001989436789;
        ThreeWheelConstants.strafeTicksToInches = 0.00295;       //default  .001989436789;
        ThreeWheelConstants.turnTicksToInches = 0.0018;       //TODO: old was TURN_TICKS_TO_RADIANS, so need to check.  default  .001989436789;
        ThreeWheelConstants.leftY = 5.25;       //default  1;
        ThreeWheelConstants.rightY =-5.25;        //default  -1;
        ThreeWheelConstants.strafeX = -2;       //default  -2.5;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftRear";       //default  "leftFront";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "rightFront";       //default  "rightRear";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightRear";       //default  "rightFront";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;    //TODO     //default  REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;    //TODO    //default  REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;   //TODO    //default  FORWARD;



        /*THIS IS FOR OTOS odometry
         *If you are using the corrected OTOS class, set OTOSConstants.useCorrectedOTOSClass to true.
         *If you are using the original OTOS class, set OTOSConstants.useCorrectedOTOSClass to false.
         *Make sure that the I2C port that you are using is the same class as the one you are using,
         * determined by above.
         */
//        OTOSConstants.useCorrectedOTOSClass = false;  //set to true or false based on above instructions
        //OTOSConstants.hardwareMapName = "OTOS";
        //OTOSConstants.linearUnit = DistanceUnit.INCH;
        //OTOSConstants.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        //OTOSConstants.offset = new SparkFunOTOS.Pose2D(0, 0, Math.PI / 2);
        //Left/right is the y axis and forward/backward is the x axis,
        // with left being positive y and forward being positive x.
        //PI/2 radians is facing forward, and clockwise rotation is negative rotation.
        //OTOSConstants.linearScalar = 1.0;         //default linear unit is DistanceUnit.INCH;  section 2.5.2a run either Forward or Lateral Localizer Tuner
        //OTOSConstants.angularScalar = 1.0;        //default angle unit is AngleUnit.RADIANS.   section 2.5.2b run Turn Localizer Tuner
            //section 2.5. Run Localization Test.  Observe the movements, moving the robot forward
            // should make x increase and strafing left should make y increase.





    }
}




