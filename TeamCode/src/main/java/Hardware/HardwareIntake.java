package Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure
//TODO: program the Servo position


public class HardwareIntake {

    public DcMotorEx intakeSlides = null;


    public CRServo leftIntakeWheel = null;
    public CRServo rightIntakeWheel = null;
    public Servo leftIntakeServo = null;
    public Servo rightIntakeServo = null;
    //Servo Test
//    public Servo Servo_Test = null;


    /*Constructor*/
    public HardwareIntake() {
    }

    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap)    {
        //Save reference to Hardware map

        //map and setup mode of Intake Slide Motor
        intakeSlides = hardwareMap.get(DcMotorEx.class, "intakeSlides");
        //intake slide motor behaviors
        intakeSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        Slider_Motor_Right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        Slider_Motor_Left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlides.setDirection(DcMotorEx.Direction.REVERSE);
        intakeSlides.setPower(0);

        //map and setup mode of Intake Continuous Servos
        leftIntakeWheel = hardwareMap.get(CRServo.class, "leftIntakeWheel");
        rightIntakeWheel = hardwareMap.get(CRServo.class, "rightIntakeWheel");
        leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");



        //TEST SERVO
//        Servo_Test = hardwareMap.get(Servo.class, "Servo_Test");

    }


    public void start(){

    }


    public void stop () {

    }

    //TODO: Method for entire intake process
    //public method (function) for intaking sample
    public void intakeSlideSetPositionPower(
            int DesiredSliderPosition,
            double SliderPower){

        intakeSlides.setTargetPosition(DesiredSliderPosition);
        intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlides.setPower(SliderPower);
    }

    //public method (function) for intaking in sample
    public void intakeIN() {
//        leftIntakeServo.setPosition(0); //TODO: tilt intake down servo positions
//        rightIntakeServo.setPosition(1);
        leftIntakeWheel.setPower(1);
        rightIntakeWheel.setPower(-1);
    }


    //public method (function) for spitting out sample
    public void intakeOUT() {
        leftIntakeWheel.setPower(-1);
        rightIntakeWheel.setPower(1);
    }

    //public method (function) for stopping the intake
    public void intakeSTOP() {
//        leftIntakeServo.setPosition(0); //TODO: tilt intake up servo positions
//        rightIntakeServo.setPosition(1);
        leftIntakeWheel.setPower(0);
        rightIntakeWheel.setPower(0);
    }

    //method for the retracted position of the intake slides
    public void intakeSlideIN() {
        intakeSlideSetPositionPower(0,0.4); //TODO: set power
    }

    //method for the extended position of the intake slides
    public void intakeSlideOUT() {
        intakeSlideSetPositionPower(500,0.4); //TODO: set position and power - old: 1700
    }
    public void intakeSlideMID() {
        intakeSlideSetPositionPower(250,0.4); //TODO: set position and power - old: 850
    }

    public void intakeDOWN(){
        leftIntakeServo.setPosition(0.81); //TODO: find correct position
        //rightIntakeServo.setPosition(0.81); //This is the actual left servo on the robot
    }
    public void intakeUP(){
        leftIntakeServo.setPosition(1); //TODO: find correct position (should be the extreme servo position - 0 or 1)
        //rightIntakeServo.setPosition(1); //This is the actual left servo on the robot
    }
    public void transferIntake(){ //TODO: finish
        intakeSTOP();
        intakeSlideIN();
        if (intakeSlides.getCurrentPosition() < 10){
            intakeOUT();
        }
    }
}