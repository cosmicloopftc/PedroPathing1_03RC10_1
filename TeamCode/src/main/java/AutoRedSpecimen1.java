
//import static org.firstinspires.ftc.teamcode.AUTOconstant.AUTOfrontIntakePickupLength;
//import static org.firstinspires.ftc.teamcode.AUTOconstant.AUTOredSample1X;
//import static org.firstinspires.ftc.teamcode.AUTOconstant.AUTOredSample1Y;
//import static org.firstinspires.ftc.teamcode.AUTOconstant.AUTOredSample2X;
//import static org.firstinspires.ftc.teamcode.AUTOconstant.AUTOredSample2Y;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import Hardware.HardwareNoDriveTrainRobot;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;
//import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.pedropathing.util.Constants;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * adapted from @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 9/8/2024
 * based on example at: https://github.com/AnyiLin/10158-Centerstage/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/competition/autonomous
 *
 * There are two major types of paths components: BezierCurves and BezierLines.
 * * BezierCurves are curved, and require > 3 points. There are the start and end points, and the control points.
 *   - Control points manipulate the curve between the start and end points.
 *   - A good visualizer for this is [this](https://www.desmos.com/calculator/3so1zx0hcd).
 * * BezierLines are straight, and require 2 points. There are the start and end points.
 * * Pose = (X,Y, Heading as Math.toRadians(degrees))...[if no Heading, then it is input as 0 degree]
 * * Point = (Pose) or = (pose.getX(), pose.getY(), Point.CARTESIAN) or = (X, Y, Point.CARTESIAN)
 *
 * Path: Represents a single movement, which can be a curve (BezierCurve) or a straight line (BezierLine).
 * PathChain: Contains Path(s) within it.
 * Interpolations: Define how the robot adjusts heading (rotation) throughout a path.
 * Timeout Constraints: Limit the time the robot spends attempting to complete a path.
 *
 *
 * 11/29/2024: start, ChainPath--start to Sample 3 and then to Net
 *             include FTC Dashboard
 * 12/2/2024 OT:  modify from PedroPathing.com auto example
 * 12/15/2024 OT: correct strafe pod anchoring; erase negative sign on straf tick/inch
 *                  and erase reverse of strafe reading; reTune all translational, drive and heading
 *                  distanceCenterToIntakePickup = 23 inches;
 *                  distanceCenterToOuttakeBasketDropOff = 13 inches
 *                  distanceCenterToSideWall = 6.5 inches
 *                  distanceCentertobackRobot = 9 inches
 *                  distanceCentertobackWallPickup = 13 inches
 *12/20/2024 MT: add subsystem actions and add pathChain to first sample pickup
 *
 *12/22/2024 OT: start this Auto Specimen-Red
 *               Add debug code
 */



@Config
@Autonomous(name = "Auto Red Specimen v1.1", group = "Auto")
public class AutoRedSpecimen1 extends OpMode {
    int debugLevel = 499;
    private Telemetry telemetryA;

    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;

    private String navigation;
    //    public ClawSubsystem claw;
    private HuskyLens huskyLens;


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    // private Pose sample1Pose, sample2Pose, sample3Pose, sample4Pose, sample5Pose, sample6Pose, redNet, blueNet;
    private PathChain preLoadSpecScore;
    private PathChain afterScore1, afterScore2, afterScore3;
    private PathChain sample1path1, sample1path2, sample1path3;
    private PathChain sample2path1, sample2path2, sample2path3;
    private PathChain sample3path1, sample3path2, sample3path3;



    HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();    //TODO: will this interfere with follower(hardwareMap)? in .init

    //private Pose startPose = new Pose(23.6 * 5 + 16, 39.75, Math.toRadians(90));  //(AUTOstartRedNetX, AUTOstartRedNetY, Math.toRadians(90));
    private Pose specScorePose = new Pose(105, 77.5, Math.toRadians(0));
    private Pose afterScorePose1 = new Pose(117, 77.5, Math.toRadians(0));
    //    private Pose pickup3Pose = new Pose(AUTOredSample3X + AUTOfrontIntakePickupLength, AUTOredSample3Y, Math.toRadians(180));
    private Pose afterScorePose2 = new Pose(117, 107.5, Math.toRadians(0));
    private Pose afterScorePose3 = new Pose(88, 107.5, Math.toRadians(0));
    private Pose sample1Pose = new Pose(88, 116.5, Math.toRadians(0));
    private Pose sample2Pose = new Pose(88, 126.5, Math.toRadians(0));
    private Pose sample3Pose = new Pose(88, 131.5, Math.toRadians(0));
    private Pose observationSample1 = new Pose(143, 116.5, Math.toRadians(0));
    private Pose observationSample2 = new Pose(143, 126.5, Math.toRadians(0));
    private Pose observationSample3 = new Pose(143, 131.5, Math.toRadians(0));
    private Pose preSpecPickup = new Pose(114, 114, Math.toRadians(180));


//    //private Pose redScorePose = new Pose(23.6 * 5 + 6, 14, Math.toRadians(135));      //(AUTORedNetX, AUTORedNetY, Math.toRadians(135));;


    private Pose startPose = new Pose(137, 77.5, Math.toRadians(0));

    //   private Pose preScorePose = new Pose(118, 17, Math.toRadians(135));
    private Pose redScorePose = new Pose(121, 14, Math.toRadians(135));

    //private Pose startPose = new Pose(144, 0, Math.toRadians(90));
    //private Pose redScorePose = new Pose(104, 0, Math.toRadians(90));
    //private Pose startPose = new Pose(144, 0, Math.toRadians(180));
    //private Pose redScorePose = new Pose(104, 0, Math.toRadians(180));



    public void buildPaths() {
        //preLoadScore = new Path(new BezierLine(new Point(startPose), new Point(redScorePose)));
        //preLoadScore.setLinearHeadingInterpolation(startPose.getHeading(), redScorePose.getHeading());

        //pathChain for preLoadSpecScore position
        preLoadSpecScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(specScorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), specScorePose.getHeading())
                .build();
        afterScore1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specScorePose), new Point(afterScorePose1)))
                .setLinearHeadingInterpolation(specScorePose.getHeading(), afterScorePose1.getHeading())
                .build();
        afterScore2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(afterScorePose1), new Point(afterScorePose2)))
                .setLinearHeadingInterpolation(afterScorePose1.getHeading(), afterScorePose2.getHeading())
                .build();
        afterScore3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(afterScorePose2), new Point(afterScorePose3)))
                .setLinearHeadingInterpolation(afterScorePose2.getHeading(), afterScorePose3.getHeading())
                .build();

//        afterScore1 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(specScorePose), new Point(afterScorePose1),
//                        new Point(afterScorePose2), new Point(afterScorePose3)))
//                .setLinearHeadingInterpolation(specScorePose.getHeading(), afterScorePose3.getHeading())
//                .build();

        sample1path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(afterScorePose3), new Point(sample1Pose)))
                .setLinearHeadingInterpolation(afterScorePose3.getHeading(), sample1Pose.getHeading())
                .build();
        sample1path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1Pose), new Point(observationSample1)))
                .setLinearHeadingInterpolation(sample1Pose.getHeading(), observationSample1.getHeading())
                .build();
        sample1path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observationSample1), new Point(sample1Pose)))
                .setLinearHeadingInterpolation(observationSample1.getHeading(), sample1Pose.getHeading())
                .build();
        sample2path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1Pose), new Point(sample2Pose)))
                .setLinearHeadingInterpolation(sample1Pose.getHeading(), sample2Pose.getHeading())
                .build();
        sample2path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2Pose), new Point(observationSample2)))
                .setLinearHeadingInterpolation(sample2Pose.getHeading(), observationSample2.getHeading())
                .build();
        sample2path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observationSample2), new Point(sample2Pose)))
                .setLinearHeadingInterpolation(observationSample2.getHeading(), sample2Pose.getHeading())
                .build();
        sample3path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2Pose), new Point(sample3Pose)))
                .setLinearHeadingInterpolation(sample2Pose.getHeading(), sample3Pose.getHeading())
                .build();
        sample3path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3Pose), new Point(observationSample3)))
                .setLinearHeadingInterpolation(sample3Pose.getHeading(), observationSample3.getHeading())
                .build();
        sample3path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observationSample3), new Point(preSpecPickup)))
                .setLinearHeadingInterpolation(observationSample3.getHeading(), preSpecPickup.getHeading())
                .build();



//        creating pathChain
//        pickupSample3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(redScorePose), new Point(pickup3Pose)))
//                .setLinearHeadingInterpolation(redScorePose.getHeading(), pickup3Pose.getHeading())            //one Heading only
//                .build();
//        preScoreSample3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup3Pose), new Point(moveForward)))
//                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), moveForward.getHeading())            //one Heading only
//                .build();
//        scoreSample3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(moveForward), new Point(redScorePose)))
//                .setLinearHeadingInterpolation(moveForward.getHeading(), redScorePose.getHeading())
//                .build();
    }






    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:     //goto basket and score
                //follower.followPath(preLoadScore);

                //TODO:start to raise outtake slider, start rotating outtake when halfway up
                autoRobot.Outtake.closeClaw();
                //        autoRobot.Outtake.highChamberSet();
                autoRobot.Intake.intakeUP();
                autoRobot.Intake.intakeSlideIN();
//                autoRobot.Outtake.leftSlideSetPositionPower(3400,1);
//                autoRobot.Outtake.rightSlideSetPositionPower(3400,1);
//                if (pathTimer.getElapsedTimeSeconds()>5){
//                    autoRobot.Outtake.highBasket();
//                }
                autoDebug(500, "Case:0 START", "Raise Slider to Specimen score");
                follower.followPath(preLoadSpecScore, false);
                setPathState(1);
                break;
            case 1:     //goto specimen 3 and pick it up
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (pathTimer.getElapsedTimeSeconds()>2) {
                    autoDebug(500, "Auto:case 1; 5 sec ", "score specimen");
                    //            autoRobot.Outtake.highChamberFinish();
                    autoDebug(500, "Auto:case 1; ", "lower slide to wall pickup");
                    follower.followPath(afterScore1, true);

                    follower.setMaxPower(1);
                    autoDebug(500, "Auto:case 1; ", "move back from Submersible");
                    setPathState(2);

                }
                break;
            case 2:     //goto specimen 3 and pick it up
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    /* Score Preload */
                    //TODO: rotate outtake and open claw/outtake to drop sample
                    follower.followPath(afterScore2, true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //then go to next path--go to Specimen 3 pickup position
//                    follower.followPath(afterScore1,false);
                    setPathState(3);
                }

                break;
            case 3:     //goto basket and score
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    /* Grab Sample */

                    //TODO: do something
                    follower.followPath(afterScore3, true);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    follower.followPath(sample1path1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    follower.followPath(sample1path2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    follower.followPath(sample1path3, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    follower.followPath(sample2path1, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    follower.followPath(sample2path2, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    follower.followPath(sample2path3, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    follower.followPath(sample3path1, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    follower.followPath(sample3path2, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    autoRobot.Outtake.wallIntake();
                    autoRobot.Outtake.openClaw();
                    follower.followPath(sample3path3, true);
                    setPathState(13);
                }
                break;


        }
        autoDebug(500, "Auto:Declaration", "DONE");
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    //**********************************************************************************
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        autoRobot.init(hardwareMap);   //TODO: check if conflicts with Follower(hardwareMap);note hardwareMap is default and part of FTC Robot Controller HardwareMap class

        autoRobot.Outtake.wallIntake();
        autoRobot.Outtake.closeClaw();

        poseUpdater = new PoseUpdater(hardwareMap);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        //  autoRobot.Outtake.groundPositionClose();
        autoRobot.Intake.intakeUP();
        autoRobot.Intake.intakeSlideIN();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

        autoDebug(500, "Auto:init", "DONE");

    }

    @Override
    public void init_loop() {


    }

    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

        autoRobot.Outtake.closeClaw();
//       autoRobot.Outtake.highChamberSet();
        autoDebug(500, "Auto:Start", "DONE");
    }

    //**********************************************************************************
    @Override
    public void loop() {
        poseUpdater.update();
        dashboardPoseTracker.update();

        follower.update();
        autonomousPathUpdate();

        follower.telemetryDebug(telemetryA);
        telemetryA.addLine("");
        telemetryA.addLine("");
        telemetryA.addData("path state", pathState);
        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        //telemetry.update();


        telemetryA.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }


    @Override
    public void stop() {
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
}



/* There are two major types of paths components: BezierCurves and BezierLines.
 *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
 *    - Control points manipulate the curve between the start and end points.
 *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
 *    * BezierLines are straight, and require 2 points. There are the start and end points.
 * Paths have can have heading interpolation: Constant, Linear, or Tangential
 *    * Linear heading interpolation:
 *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
 *    * Constant Heading Interpolation:
 *    - Pedro will maintain one heading throughout the entire path.
 *    * Tangential Heading Interpolation:
 *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
 * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
 * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html>
 */




//    public void buildPaths() {
//        Pose sample1Pose = new Pose(AUTOredSample1X, AUTOredSample1Y);
//        startSample1 = new Path(new BezierLine(new Point(startPose), new Point(sample1Pose);
//        //Sample1.setLinearHeadingInterpolation(startSample1.getHeading(), initialBackdropGoalPose.getHeading());
//        startSample1.setPathEndTimeoutConstraint(0);
//
//        /** This is a path chain, defined on line 66
//         * It, well, chains multiple paths together. Here we use a constant heading from the board to the stack.
//         * On line 97, we set the Linear Interpolation,
//         * which means that Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path */
//
////        cycleStackTo = follower.pathBuilder()
////                .addPath(new BezierLine(new Point(initialBackdropGoalPose), new Point(TopTruss)))
////                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
////                .addPath(new BezierLine(new Point(TopTruss), new Point(BottomTruss)))
////                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
////                .addPath(new BezierCurve(new Point(BottomTruss), new Point(12 + 13 + 1, 12, Point.CARTESIAN), new Point(31 + 12 + 1, 36, Point.CARTESIAN), new Point(Stack)))
////                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
////                .setPathEndTimeoutConstraint(0)
////                .build();
//
////        cycleStackBack = follower.pathBuilder()
////                .addPath(new BezierLine(new Point(Stack), new Point(BottomTruss)))
////                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
////                .addPath(new BezierLine(new Point(BottomTruss), new Point(TopTruss)))
////                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
////                .addPath(new BezierLine(new Point(TopTruss), new Point(WhiteBackdrop)))
////                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
////                .setPathEndTimeoutConstraint(0)
////                .build();
//
//    }