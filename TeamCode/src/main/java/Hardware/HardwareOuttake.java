package Hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure

public class HardwareOuttake {
    //    private DcMotor Intake_Motor = null;
    public DcMotorEx outtakeLeftSlide = null;
    public DcMotorEx outtakeRightSlide = null;

    public Servo outtakeArmAxon = null;
    public Servo outtakeExtension = null;
    public Servo claw = null;
    /*Constructor*/
    public HardwareOuttake() {
    }

    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap)    {
        //Save reference to Hardware map
        //map and setup mode of slide motors
        outtakeLeftSlide = hardwareMap.get(DcMotorEx.class, "outtakeLeftSlide");
        outtakeLeftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeLeftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeLeftSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeLeftSlide.setDirection(DcMotorEx.Direction.FORWARD); //It is forward on robot
        outtakeLeftSlide.setPower(0);

        outtakeRightSlide = hardwareMap.get(DcMotorEx.class, "outtakeRightSlide");
        outtakeRightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeRightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeRightSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeRightSlide.setDirection(DcMotorEx.Direction.REVERSE); //It is reversed on robot
        outtakeRightSlide.setPower(0);

        outtakeArmAxon = hardwareMap.get(Servo.class, "outtakeArmAxon");
        outtakeExtension = hardwareMap.get(Servo.class, "outtakeExtension");
        claw = hardwareMap.get(Servo.class, "claw");


    }


    public void start(){

    }


    public void stop () {

    }
    public void leftSlideSetPositionPower(
            int DesiredSliderPosition,
            double SliderPower){

        outtakeLeftSlide.setTargetPosition(DesiredSliderPosition);
        outtakeLeftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeLeftSlide.setPower(SliderPower);
    }
    public void rightSlideSetPositionPower(
            int DesiredSliderPosition,
            double SliderPower){

        outtakeRightSlide.setTargetPosition(DesiredSliderPosition);
        outtakeRightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeRightSlide.setPower(SliderPower);
    }


    public void openClaw(){
        claw.setPosition(0.17); // TODO: find new position
    }
    public void closeClaw(){
        claw.setPosition(0.32); // TODO: find new position
    }
    public void groundPositionOpen(){
        leftSlideSetPositionPower(0,1);
        rightSlideSetPositionPower(0,1);
        //outtakeArmAxon.setPosition(0); //TODO: Find position for transfering (stays the same throughout process)
        //outtakeExtension.setPosition(0); //TODO: Find the no extension position
        openClaw();
    }
    public void groundPositionClose(){
        leftSlideSetPositionPower(0,0);
        rightSlideSetPositionPower(0,0);
        //outtakeArmAxon.setPosition(0); //Figure out position for transfering (stays the same throughout process)
        //outtakeExtension.setPosition(0); //Find the no extension position
        closeClaw();
    }

    public void readyPosition(){
        //outtakeArmAxon.setPosition(0); //Should be same as ground position
        //outtakeExtension.setPosition(0); //Should be same as ground position
        leftSlideSetPositionPower(500,1);
        rightSlideSetPositionPower(500,1);
    }

    public void lowBasket(){
        //outtakeArmAxon.setPosition(0); //Should be same as high basket
        //outtakeExtension.setPosition(0); //No extension
        leftSlideSetPositionPower(970,0.6);
        rightSlideSetPositionPower(970,0.6);
    }
    public void highBasket(){
        //outtakeArmAxon.setPosition(0); //TODO: Find position
        //outtakeExtension.setPosition(0); //No extension TODO: Find position
        leftSlideSetPositionPower(3400,1);
        rightSlideSetPositionPower(3400,1);
    }
    public void lowChamber(){ //Not able to do this with current V1 robot
//        leftSlideSetPositionPower(0,0);
//        rightSlideSetPositionPower(0,0);
//        leftOuttakeArm.setPosition(0);
//        rightOuttakeArm.setPosition(1);
//        claw.setPosition(0);
    }
    public void highChamberSetFront(){
        leftSlideSetPositionPower(700,0.6); //Find position
        rightSlideSetPositionPower(700,0.6); //Find position
        //outtakeArmAxon.setPosition(0); //Find position
        //outtakeExtension.setPosition(0); //All extended
    }
    public void highChamberFinishFront(){
        leftSlideSetPositionPower(700,1); //Find position (Go up from high chamber set position)
        rightSlideSetPositionPower(700,1);
        //outtakeArmAxon.setPosition(0); //Find position -- same as high chamber set position
        //outtakeExtension.setPosition(0); //All extended
        openClaw();
    }
    public void highChamberSetBack(){
        leftSlideSetPositionPower(700,0.6); //Find position
        rightSlideSetPositionPower(700,0.6); //Find position
        //outtakeArmAxon.setPosition(0); //Should be same as wall intake
        //outtakeExtension.setPosition(0); //All extended
    }
    public void highChamberFinishBack(){
        leftSlideSetPositionPower(700,1); //Find position (Go down from high chamber set position)
        rightSlideSetPositionPower(700,1);
        //outtakeArmAxon.setPosition(0); //Should be same as wall intake
        //outtakeExtension.setPosition(0); //No extension
        openClaw();
    }
    public void wallIntake(){
        leftSlideSetPositionPower(0,1);
        rightSlideSetPositionPower(0,1);
        //outtakeArmAxon.setPosition(0); //TODO: Find position
        //outtakeExtension.setPosition(0); //TODO: Find position
    }

    public double getOuttakeSliderRightCurrent(){
        return outtakeRightSlide.getCurrent(CurrentUnit.MILLIAMPS);
    }
    public double getOuttakeSliderLeftCurrent(){
        return outtakeLeftSlide.getCurrent(CurrentUnit.MILLIAMPS);
    }

}