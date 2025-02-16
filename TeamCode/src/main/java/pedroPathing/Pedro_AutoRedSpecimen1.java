package pedroPathing;
//import static org.firstinspires.ftc.teamcode.RR.AUTOstorageConstant.AUTOfrontIntakePickupLength;
//import static org.firstinspires.ftc.teamcode.RR.AUTOstorageConstant.AUTOredSample1X;
//import static org.firstinspires.ftc.teamcode.RR.AUTOstorageConstant.AUTOredSample1Y;
//import static org.firstinspires.ftc.teamcode.RR.AUTOstorageConstant.AUTOredSample2X;
//import static org.firstinspires.ftc.teamcode.RR.AUTOstorageConstant.AUTOredSample2Y;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.pedropathing.util.Constants;

import RR.AUTOstorageConstant;
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
 *
 *2/16/2025     Pedro_RedBasket_Feb16_2025_MoreCurvePath_19second
 */



@Config
@Autonomous(name = "Pedro Auto Red Specimen v1.1", group = "Auto")
public class Pedro_AutoRedSpecimen1 extends OpMode {
    String AllianceBasketOrSpecimen = "1RedSpecimen";
    private double loopTimeTotal, loopTimeCount;
    private ElapsedTime loopTime = new ElapsedTime();


    int debugLevel = 501;
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
    private PathChain preLoadSpecScore, bringSample1Home, bringSample2Home,bringSample3Home;
    private PathChain sample3HomeToSpecimenPickup;
    private PathChain scoreSpecimen1, scoreSpecimen2, scoreSpecimen3, scoreSpecimen4;
    private PathChain returnHomeAfterScoringSpecimen1, returnHomeAfterScoringSpecimen2,
            returnHomeAfterScoringSpecimen3, returnHomeAfterScoringSpecimen4;


    HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();    //TODO: will this interfere with follower(hardwareMap)? in .init

    //private Pose startPose = new Pose(23.6 * 5 + 16, 39.75, Math.toRadians(90));  //(AUTOstartRedNetX, AUTOstartRedNetY, Math.toRadians(90));
    private Pose startPose = new Pose(7.5, 64, Math.toRadians(180));
    private Pose preloadScorePose = new Pose(33.5, 64, Math.toRadians(180));
    private Pose ScorePose1 = new Pose(33.5, 68, Math.toRadians(180));
    //    private Pose pickup3Pose = new Pose(AUTOredSample3X + AUTOfrontIntakePickupLength, AUTOredSample3Y, Math.toRadians(180));
    private Pose ScorePose2 = new Pose(33.5, 72, Math.toRadians(180));
    private Pose ScorePose3 = new Pose(33.5, 76, Math.toRadians(180));
    private Pose ScorePose4 = new Pose(33.5, 80, Math.toRadians(180));

    private Pose sample1HomePose = new Pose(18, 25, Math.toRadians(180));
    private Pose sample2HomePose = new Pose(18, 16, Math.toRadians(180));
    private Pose sample3HomePose = new Pose(18, 7.5, Math.toRadians(180));

    private Pose specimenPickupPose = new Pose(7.5, 31, Math.toRadians(180));


    public void buildPaths() {
        preLoadSpecScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadScorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadScorePose.getHeading())
                .build();
        bringSample1Home = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadScorePose),
                        new Point(19,55),
                        new Point(7,32),
                        new Point(126,27),
                        new Point(sample1HomePose)
                        ))
                .setLinearHeadingInterpolation(preloadScorePose.getHeading(), sample1HomePose.getHeading())
                .build();
        bringSample2Home = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sample1HomePose),
                        new Point(107,15),
                        new Point(sample2HomePose)
                ))
                .setLinearHeadingInterpolation(sample1HomePose.getHeading(), sample2HomePose.getHeading())
                .build();
        bringSample3Home = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sample2HomePose),
                        new Point(107,6),
                        new Point(sample3HomePose)
                ))
                .setLinearHeadingInterpolation(sample2HomePose.getHeading(), sample3HomePose.getHeading())
                .build();

        sample3HomeToSpecimenPickup = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sample3HomePose),
                        new Point(19,31),
                        new Point(19,31),
                        new Point(specimenPickupPose)
                ))
                .setLinearHeadingInterpolation(sample3HomePose.getHeading(), specimenPickupPose.getHeading())
                .build();

        scoreSpecimen1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenPickupPose),
                        new Point(16,68),
                        new Point(22,68),
                        new Point(ScorePose1)
                ))
                .setLinearHeadingInterpolation(specimenPickupPose.getHeading(), ScorePose1.getHeading())
                .build();
        returnHomeAfterScoringSpecimen1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(ScorePose1),
                        new Point(23,54),
                        new Point(12,68),
                        new Point(30,37),
                        new Point(20,31),
                        new Point(specimenPickupPose)
                ))
                .setLinearHeadingInterpolation(ScorePose1.getHeading(), specimenPickupPose.getHeading())
                .build();

        scoreSpecimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenPickupPose),
                        new Point(16,72),
                        new Point(22,72),
                        new Point(ScorePose2)
                ))
                .setLinearHeadingInterpolation(specimenPickupPose.getHeading(), ScorePose2.getHeading())
                .build();
        returnHomeAfterScoringSpecimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(ScorePose2),
                        new Point(23,54),
                        new Point(12,72),
                        new Point(30,37),
                        new Point(20,31),
                        new Point(specimenPickupPose)
                ))
                .setLinearHeadingInterpolation(ScorePose2.getHeading(), specimenPickupPose.getHeading())
                .build();

        scoreSpecimen3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenPickupPose),
                        new Point(16,76),
                        new Point(22,76),
                        new Point(ScorePose3)
                ))
                .setLinearHeadingInterpolation(specimenPickupPose.getHeading(), ScorePose3.getHeading())
                .build();
        returnHomeAfterScoringSpecimen3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(ScorePose3),
                        new Point(23,54),
                        new Point(12,76),
                        new Point(30,37),
                        new Point(20,31),
                        new Point(specimenPickupPose)
                ))
                .setLinearHeadingInterpolation(ScorePose3.getHeading(), specimenPickupPose.getHeading())
                .build();

        scoreSpecimen4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenPickupPose),
                        new Point(16,80),
                        new Point(22,80),
                        new Point(ScorePose4)
                ))
                .setLinearHeadingInterpolation(specimenPickupPose.getHeading(), ScorePose4.getHeading())
                .build();
        returnHomeAfterScoringSpecimen4 = follower.pathBuilder()                //go straight back
                .addPath(new BezierCurve(new Point(ScorePose4),
                        new Point(15,51),
                        new Point(17,48),
                        new Point(specimenPickupPose)
                ))
                .setLinearHeadingInterpolation(ScorePose4.getHeading(), specimenPickupPose.getHeading())
                .build();

    }






    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:     //goto basket and score
                //follower.followPath(preLoadScore);

                //TODO:start to raise outtake slider, start rotating outtake when halfway up
//                autoRobot.Outtake.closeClaw();
                //        autoRobot.Outtake.highChamberSet();
//                autoRobot.Intake.intakeUP();
//                autoRobot.Intake.intakeSlideIN();
//                autoRobot.Outtake.leftSlideSetPositionPower(3400,1);
//                autoRobot.Outtake.rightSlideSetPositionPower(3400,1);
//                if (pathTimer.getElapsedTimeSeconds()>5){
//                    autoRobot.Outtake.highBasket();
//                }
                autoDebug(500, "Case: 0 START", "Raise Slider to Specimen score");
                follower.followPath(preLoadSpecScore, true);
                setPathState(1);
                break;
            case 1:     //Goto and push Sample 1 Home
                     /* You could check for
                    - Follower State: "if(!follower.isBusy() {}"
                    - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                    - Robot Position: "if(follower.getPose().getX() > 36) {}"
                    */
                if (!follower.isBusy()) {
                    //TODO: open claw here--?need to pause to open before moving
                    autoDebug(500, "Auto: case 1: ", "score preLoad");
                    follower.followPath(bringSample1Home, false);
                  //  follower.setMaxPower(0.8);
                    autoDebug(500, "Auto: case 1: ", "move sample 1 Home");
                    setPathState(2);
                }
                break;

            case 2:     //rotate arm into robot, lower slider, extend arm to wall pick position
                        // after ?1 second of starting to head to Sample 1.
                        //THEN move sample 2 home
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    //TODO: rotate arm into robot, lower slider, extend arm to wall pick position after ?1 second
                }
                if (!follower.isBusy()) {
                    follower.followPath(bringSample2Home, false);
                    autoDebug(500, "Auto: case 2: ", "move sample 2 Home");
                    setPathState(3);
                }
                break;
            case 3:     //THEN move sample 3 home
                if (!follower.isBusy()) {
                    follower.followPath(bringSample3Home, false);
                    autoDebug(500, "Auto: case 3: ", "move sample 3 Home");
                    setPathState(4);
                }
                break;

            case 4:     //THEN move sample 3 home
                if (!follower.isBusy()) {
                    follower.followPath(sample3HomeToSpecimenPickup, true);
                    autoDebug(500, "Auto: case 4: ", "pickup Specimen 1 at Wall");
                    setPathState(5);
                }
                break;


            case 5:     //Grab Specimen 1 at Wall
                        // and THEN go to Submersible pole
                if (!follower.isBusy()) {
                    //TODO: close claw, does it need 0.2 second hold for the claw to grab before moving robot?
                    follower.followPath(scoreSpecimen1, true);
                    autoDebug(500, "Auto: case 5: ", "goto Submersible, score Spec1");
                    setPathState(6);
                }
                break;
            case 6:     //raise slider, rotate arm out, retract arm for scoring Submersible position as robot leaves WaLL
                        // and THEN goto Home
                    //TODO: raise slider, rotate arm out, retract arm for scoring Submersible position as robot leaves Wall
                if (!follower.isBusy()) {
                    //TODO: open claw here--?need to pause to open before moving
                    autoDebug(500, "Auto: case 6: ", "score spec 1");
                    follower.followPath(returnHomeAfterScoringSpecimen1, true);
                    autoDebug(500, "Auto: case 6: ", "return Home");
                    setPathState(7);
                }
                break;


            case 7:     //After ?0.5 second from Submersible for Home,
                        // rotate arm into robot, lower slider, extend arm to wall pick position
                        // continue to Home
                        // and THEN close claw, and go to Submersible pole to score specimen 2
                if (pathTimer.getElapsedTimeSeconds()>0.5) {
                    //TODO: rotate arm into robot, lower slider, extend arm to wall pick position after ?0.5 second
                }
                if (!follower.isBusy()) {
                    //TODO: close claw, does it need to hold 0.2 second for the claw to grab specimen before moving robot?
                    follower.followPath(scoreSpecimen2, true);
                    autoDebug(500, "Auto:case 7: ", "goto Submersible, score Spec2");
                    setPathState(8);
                }
                break;
            case 8:     //raise slider, rotate arm out, retract arm for scoring Submersible position
                    //TODO: raise slider, rotate arm out, retract arm for scoring Submersible position
                if (!follower.isBusy()) {
                    //TODO: open claw here--?need to pause to open before moving
                    follower.followPath(returnHomeAfterScoringSpecimen2, true);
                    autoDebug(500, "Auto:case 8: ", "return Home");
                    setPathState(8);
                }
                break;


            case 9:     //After ?0.5 second from Submersible for Home,
                // rotate arm into robot, lower slider, extend arm to wall pick position
                // continue to Home
                // and THEN close claw, and go to Submersible pole to score specimen 2
                if (pathTimer.getElapsedTimeSeconds()>0.5) {
                    //TODO: rotate arm into robot, lower slider, extend arm to wall pick position after ?0.5 second
                }
                if (!follower.isBusy()) {
                    //TODO: close claw, does it need to hold 0.2 second for the claw to grab specimen before moving robot?
                    follower.followPath(scoreSpecimen3, true);
                    autoDebug(500, "Auto: case 9: ", "goto Submersible, score Spec3");
                    setPathState(10);
                }
                break;
            case 10:     //raise slider, rotate arm out, retract arm for scoring Submersible position
                //TODO: raise slider, rotate arm out, retract arm for scoring Submersible position
                if (!follower.isBusy()) {
                    //TODO: open claw here--?need to pause to open before moving
                    follower.followPath(returnHomeAfterScoringSpecimen3, true);
                    autoDebug(500, "Auto: case 10: ", "return Home");
                    setPathState(11);
                }
                break;


            case 11:     //After ?0.5 second from Submersible for Home,
                // rotate arm into robot, lower slider, extend arm to wall pick position
                // continue to Home
                // and THEN close claw, and go to Submersible pole to score specimen 2
                if (pathTimer.getElapsedTimeSeconds()>0.5) {
                    //TODO: rotate arm into robot, lower slider, extend arm to wall pick position after ?0.5 second
                }
                if (!follower.isBusy()) {
                    //TODO: close claw, does it need to hold 0.2 second for the claw to grab specimen before moving robot?
                    follower.followPath(scoreSpecimen4, true);
                    autoDebug(500, "Auto: case 11: ", "goto Submersible, score Spec4");
                    setPathState(12);
                }
                break;
            case 12:     //raise slider, rotate arm out, retract arm for scoring Submersible position
                //TODO: raise slider, rotate arm out, retract arm for scoring Submersible position
                if (!follower.isBusy()) {
                    //TODO: open claw here--?need to pause to open before moving
                    follower.followPath(returnHomeAfterScoringSpecimen4, true);
                    autoDebug(500, "Auto: case 12: ", "return Home");
                    setPathState(13);
                }
                break;

            case 13:     //After ?0.5 second from Submersible for Home,
                // rotate arm into robot, lower slider, extend arm to wall pick position
                // continue to Home
                if (pathTimer.getElapsedTimeSeconds()>0.5) {
                    //TODO: rotate arm into robot, lower slider, extend arm to wall pick position after ?0.5 second
                }
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
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

//        autoRobot.Outtake.wallIntake();
//        autoRobot.Outtake.closeClaw();

        poseUpdater = new PoseUpdater(hardwareMap);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        //  autoRobot.Outtake.groundPositionClose();
//        autoRobot.Intake.intakeUP();
//        autoRobot.Intake.intakeSlideIN();
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

//        autoRobot.Outtake.closeClaw();
//       autoRobot.Outtake.highChamberSet();
        autoDebug(500, "Auto:Start", "DONE");
    }

    //**********************************************************************************
    @Override
    public void loop() {
        loopTime.reset();

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

        /**CONTINUOUSLY TRANSFER SUBSYSTEM positions at end of AUTO TO TELEOP */
        AUTOstorageConstant.AllianceBasketOrSpecimen = AllianceBasketOrSpecimen;
        //Intake servo and motor position position:
        AUTOstorageConstant.autoEnd_Intake_intakeSlides_MotorPosition = autoRobot.Intake.intakeSlides.getCurrentPosition();
        AUTOstorageConstant.autoEnd_Intake_intakeServoAxon_ServoPosition = autoRobot.Intake.intakeServoAxon.getPosition();  //TODO: confirm name
        //Outtake servo and motor position:
        AUTOstorageConstant.autoEnd_Outtake_outtakeLeftSlide_MotorPosition = autoRobot.Outtake.outtakeLeftSlide.getCurrentPosition();
        AUTOstorageConstant.autoEnd_Outtake_outtakeRightSlide_MotorPosition = autoRobot.Outtake.outtakeRightSlide.getCurrentPosition();
        AUTOstorageConstant.autoEnd_Outtake_outtakeArmAxon_ServoPosition = autoRobot.Outtake.outtakeArmAxon.getPosition();  //TODO: confirm name
        AUTOstorageConstant.autoEnd_Outtake_claw_ServoPosition = autoRobot.Outtake.claw.getPosition();
        //Drivetrain: Pose and heading
        AUTOstorageConstant.autoEndheadingIMU_yawDEG = autoRobot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);;
        AUTOstorageConstant.autoPoseEnd =  new Pose2d(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading());
        AUTOstorageConstant.autoEndX = follower.getPose().getX();
        AUTOstorageConstant.autoEndY = follower.getPose().getY();
        AUTOstorageConstant.autoEndHeadingDEG = Math.toDegrees(follower.getPose().getHeading());


        loopTimeTotal = loopTimeTotal + loopTime.milliseconds();
        loopTimeCount = loopTimeCount + 1;
        telemetryA.addData("Average loop Time (ms) = ", "%.3f", loopTimeTotal/loopTimeCount);

        telemetryA.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }


    @Override
    public void stop() {
        /**TRANSFER SUBSYSTEM positions at end of AUTO TO TELEOP */
        AUTOstorageConstant.AllianceBasketOrSpecimen = AllianceBasketOrSpecimen;
        //Intake servo and motor position position:
        AUTOstorageConstant.autoEnd_Intake_intakeSlides_MotorPosition = autoRobot.Intake.intakeSlides.getCurrentPosition();
        AUTOstorageConstant.autoEnd_Intake_intakeServoAxon_ServoPosition = autoRobot.Intake.intakeServoAxon.getPosition();  //TODO: confirm name
        //Outtake servo and motor position:
        AUTOstorageConstant.autoEnd_Outtake_outtakeLeftSlide_MotorPosition = autoRobot.Outtake.outtakeLeftSlide.getCurrentPosition();
        AUTOstorageConstant.autoEnd_Outtake_outtakeRightSlide_MotorPosition = autoRobot.Outtake.outtakeRightSlide.getCurrentPosition();
        AUTOstorageConstant.autoEnd_Outtake_outtakeArmAxon_ServoPosition = autoRobot.Outtake.outtakeArmAxon.getPosition();  //TODO: confirm name
        AUTOstorageConstant.autoEnd_Outtake_claw_ServoPosition = autoRobot.Outtake.claw.getPosition();
        //Drivetrain: Pose and heading
        AUTOstorageConstant.autoEndheadingIMU_yawDEG = autoRobot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);;
        AUTOstorageConstant.autoPoseEnd =  new Pose2d(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading());
        AUTOstorageConstant.autoEndX = follower.getPose().getX();
        AUTOstorageConstant.autoEndY = follower.getPose().getY();
        AUTOstorageConstant.autoEndHeadingDEG = Math.toDegrees(follower.getPose().getHeading());
    }

    //Debugging messages
    void autoDebug(int myLevel, String myName, String myMessage) {
        if (debugLevel > myLevel)  {
            telemetryA.addData("**DEBUG**: " + myName, myMessage);
            //telemetryA.update();
        }
        if ((1000 > myLevel) || (debugLevel > myLevel)) {
            RobotLog.i("**DEBUG** == " + myName + ": " + myMessage);
        }
    }

    //Log messages that are always shown
    void autoLog(String myName, String myMessage){
        telemetryA.addData(myName, myMessage);
        //telemetryA.update();
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