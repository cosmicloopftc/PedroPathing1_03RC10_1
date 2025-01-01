package Hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure
//TODO: program the Servo position


public class HardwareTestDevice {
    public Servo servoTest = null;
    public DcMotorEx motorTest = null;

    /*Constructor*/
    public HardwareTestDevice() {
    }

    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap)    {
        //Save reference to Hardware map
        servoTest = hardwareMap.get(Servo.class, "servoTest");
        motorTest = hardwareMap.get(DcMotorEx.class, "motorTest");

//        //map and setup mode of Intake Slide Motor
//        intakeSlides = hardwareMap.get(DcMotorEx.class, "intakeSlides");
//        //intake slide motor behaviors
//        intakeSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        intakeSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        intakeSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        Slider_Motor_Right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        intakeSlides.setDirection(DcMotorEx.Direction.REVERSE);
//        intakeSlides.setPower(0);
    }

    public void start(){
    }

    public void stop () {
    }

}