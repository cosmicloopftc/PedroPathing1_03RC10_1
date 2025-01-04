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
    public DcMotorEx intakeSpinnerMotor = null;


    public Servo intakeServo = null;
    public Servo intakeLatch = null;
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

        intakeSpinnerMotor = hardwareMap.get(DcMotorEx.class, "intakeSpinnerMotor");
        //intake spin motor behaviors
        intakeSpinnerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //intakeSpinnerMotor.setDirection(DcMotorEx.Direction.REVERSE); //TODO: Reverse or not?
        intakeSlides.setPower(0);

        //map and setup mode of Intake Servos
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeLatch = hardwareMap.get(Servo.class, "intakeLatch");



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
        intakeSpinnerMotor.setPower(0.5);
        //intakeServo.setPosition(0); //TODO: Position where intake is down
        //intakeLatch.setPosition(0); // TODO: figure out position
    }


    //public method (function) for spitting out sample
    public void intakeOUT() {
        intakeSpinnerMotor.setPower(-0.5);
        //intakeServo.setPosition(0); //TODO: Position where intake is partially down
        //intakeLatch.setPosition(0); // TODO: figure out position
    }

    //public method (function) for stopping the intake
    public void intakeSTOP() {
        intakeSpinnerMotor.setPower(0);
        //intakeServo.setPosition(0); //TODO: Position where intake is partially down
        //intakeLatch.setPosition(0); // TODO: figure out position
    }

    //method for the retracted position of the intake slides
    public void intakeSlideIN() {
        intakeSlideSetPositionPower(0,0.4); //TODO: set power
    }

    //method for the extended position of the intake slides
    public void intakeSlideOUT() {
        intakeSlideSetPositionPower(500,0.4); //TODO: find position and power
    }
    public void intakeSlideMID() {
        intakeSlideSetPositionPower(250,0.4); //TODO: find position and power
    }

    public void intakeDOWN(){
        //intakeServo.setPosition(0); //TODO: find correct position
    }
    public void intakeUP(){
        //intakeServo.setPosition(0); //TODO: find correct position
    }
    public void transferIntake(){ //TODO: figure out how this is gonna work
        //intakeLatch.setPosition(0); // TODO: figure out position
    }
}