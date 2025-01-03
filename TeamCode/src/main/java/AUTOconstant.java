
/**
 * Simple static field serving as a storage medium for the bot's pose and other CONSTANT
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 * **INTO THE DEEP 2024-2025, PedroPathing Coordinate Plane
 *   (x,y) starts at (0,0) at bottom left on Blue Alliance Observation side
 *      Pose Angle 0 degree start here--point to right toward Red Net side
 *    *Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
 *     (For Centerstage, this would be blue far side/red human player station.)
 *     (x,y) in RoadRunner transpose to (y+72,x-72)
 *     incremental mat border:  0,24,48,72,96,120,144
 *     sample dimension = 1.5 inch x 3.5 inch
 *   updated 11/27/2028: start coordinate mapping
 */


public class AUTOconstant {
//    public static String startingAllianceLocation;
//    // static Pose2d currentPose = new Pose2d();
//    public static double autoEndX;
//    public static double autoEndY;
//    public static double autoEndHeadingRadian;
//
//    public static int autoEnd_Slider_MotorPosition;
//    public static double autoEnd_Slider_ServoArmPosition;
//    public static double autoEnd_Slider_ServoWristPosition;
//    public static double autoEnd_Slider_ServoDropperPosition;
//
//    public static double autoEnd_Airplane_ServoPosition;
//
//    public static double autoEnd_Intake_ServoPosition;      //drop off Purple pixel to spike mark
//    public static double autoEnd_Intake_ServoLeftPosition;
//    public static double autoEnd_Intake_ServoRightPosition;
//    public static double autoEndheadingIMU_yawDEG;

    public static double AUTOrobotWidth = 14;    //TODO: might be 16 in       //effective inches
    public static double AUTOrobotLength = 13;         //effective inches
    public static double AUTObackOuttakeDropOffLength = 9;
    public static double AUTOfrontIntakePickupLength = 9;        //distance of outtake from center when picking speciman;

    public static double AUTOstartRedNetX = 144 - AUTOrobotLength/2;;
    public static double AUTOstartRedNetY= 24 - AUTOrobotWidth/2;    //start = right side panel is at mat line

    //below is for PedroPathing Coordinate of center of Sample
    //red Basket start     //TODO--Need to confirm coordinates
    public static double AUTOredSample1X = 98.25;             //-71 + 72;      //-(AUTOblueSample6Y);
    public static double AUTOredSample1Y =  2.75;            //-23.625 + 72;  //-(AUTOblueSample6X);
    public static double AUTOredSample2X = 98.25;              //-59.25 + 72;     //-(AUTOblueSample5Y);
    public static double AUTOredSample2Y = 12.75;           //-23.625 + 72;    //-(AUTOblueSample5X);
    public static double AUTOredSample3X = 98.25;               //-47.5 + 72;    //-(AUTOblueSample4Y);
    public static double AUTOredSample3Y = 22.5;            //-23.625 + 72;  //-(AUTOblueSample4X);
    public static double AUTORedNetX = 130.00;           // previsouly: 139              //-71 + 72;
    public static double AUTORedNetY =   14;        //previously: 5               //-58 + 72;

    //Red Observation start //TODO--Need replacement
    public static double AUTOredSample4X = 47.5 + 72;    //-(AUTOblueSample1Y);
    public static double AUTOredSample4Y = -23.625 + 72; //-(AUTOblueSample1X);
    public static double AUTOredSample5X = 59.25 + 72;     //-(AUTOblueSample2Y);
    public static double AUTOredSample5Y = -23.625 + 72;   //-(AUTOblueSample2X);
    public static double AUTOredSample6X = 71 + 72;      //-(AUTOblueSample3Y);
    public static double AUTOredSample6Y = -23.625 +72;  //-(AUTOblueSample3X);

    // Blue Net start       //TODO--Need replacement
    public static double AUTOblueSample1X =71 + 72;
    public static double AUTOblueSample1Y =23.625 + 72;
    public static double AUTOblueSample2X =59.25 + 72;
    public static double AUTOblueSample2Y =23.625 + 72;
    public static double AUTOblueSample3X =47.5 + 72;
    public static double AUTOblueSample3Y =23.625 + 72;
    public static double AUTOblueNetX =71 + 72;
    public static double AUTOblueNetY =58 + 72;

    //Blue Observation start        //TODO--Need replacement
    public static double AUTOblueSample6X =-71 + 72;
    public static double AUTOblueSample6Y =23.625 + 72;
    public static double AUTOblueSample5X =-59.25 + 72;
    public static double AUTOblueSample5Y =23.625 + 72;
    public static double AUTOblueSample4X =-47.5 + 72;
    public static double AUTOblueSample4Y =23.625 + 72;






}