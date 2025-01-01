package Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure

public class HardwareGamePadLED {
//    private DcMotor Intake_Motor = null;

    //Rumble END GAME related WARNINGS JAVA FOR FTC book
    Gamepad.RumbleEffect customRumbleEffect;
    boolean endGameRumble45secondsWarningOnce = true;
    boolean endGameRumble31secondSTARTonce = true;
    boolean endGameRumble20secondsLeftOnce = true;
    boolean endGameRumble6secondsLeftOnce = true;
    Gamepad.LedEffect flashingWhite6Sec = new Gamepad.LedEffect.Builder()
            .addStep(1, 1, 1, 500) // Show white for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 1, 1, 500) // Show white for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 1, 1, 500) // Show white for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 1, 1, 500) // Show white for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 1, 1, 500) // Show white for 500ms
            .addStep(0, 0, 0, 500)
            .build();

    Gamepad.LedEffect flashingBlue6Sec = new Gamepad.LedEffect.Builder()
            .addStep(0, 0, 1, 500) // Show blue for 500ms
            .addStep(0, 0, 0, 500) //
            .addStep(0, 0, 1, 500) // Show blue for 500ms
            .addStep(0, 0, 0, 500) //
            .addStep(0, 0, 1, 500) // Show blue for 500ms
            .addStep(0, 0, 0, 500) //
            .addStep(0, 0, 1, 500) // Show blue for 500ms
            .addStep(0, 0, 0, 500) //
            .addStep(0, 0, 1, 500) // Show blue for 500ms
            .addStep(0, 0, 0, 500) //
            .addStep(0, 0, 1, 500) // Show blue for 500ms
            .addStep(0, 0, 0, 500) //
//                .addStep(1, 0, 0, 250) // Show red for 250ms
//                .addStep(0, 1, 0, 250) // Show green for 250ms
//                .addStep(0, 0, 1, 250) // Show blue for 250ms
//                .addStep(1, 1, 1, 250) // Show white for 250ms
            .build();
    Gamepad.LedEffect flashingRed6Sec = new Gamepad.LedEffect.Builder()
            .addStep(1, 0, 0, 500) // Show red for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 0, 0, 500) // Show red for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 0, 0, 500) // Show red for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 0, 0, 500) // Show red for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 0, 0, 500) // Show red for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 0, 0, 500) // Show red for 500ms
            .addStep(0, 0, 0, 500)
            .build();


    /*Constructor*/
    public HardwareGamePadLED() {
    }

    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap)    {
        //Save reference to Hardware map


    }


    public void start(){

    }


    public void stop () {

    }

//example:  for Intake motor
//public method (function) for Intake motor power--to be accessible from anywhere
//    public void setPower(double i) {
//        Intake_Motor.setPower(i);
//    }

    public void rumble(){
        Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .build();
        gamepad1.runRumbleEffect(customRumbleEffect);
        gamepad2.runRumbleEffect(customRumbleEffect);
    }

    public void rumble6sec(){
        Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec

                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .build();
        gamepad1.runRumbleEffect(customRumbleEffect);
        gamepad2.runRumbleEffect(customRumbleEffect);
    }




}