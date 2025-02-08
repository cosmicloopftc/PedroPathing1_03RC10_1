package Hardware;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.w8wjb.ftc.AdafruitNeoDriver;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareLED {
    //public AdafruitNeoDriver neopixels;
    public DigitalChannel LEDrightGreen;
    public DigitalChannel LEDrightRed;
    public DigitalChannel LEDleftRed;
    public DigitalChannel LEDleftGreen;
    public DigitalChannel LEDright2Green;
    public DigitalChannel LEDright2Red;
    public DigitalChannel LEDleft2Red;
    public DigitalChannel LEDleft2Green;
    public ColorSensor colorSensor;
    public SparkFunLEDStick ledStick;


    public HardwareLED() {

    }
    public void init(HardwareMap hardwareMap) {

        LEDleftGreen = hardwareMap.get(DigitalChannel.class, "leftgreen");               //connect to Digital port 0
        LEDleftRed = hardwareMap.get(DigitalChannel.class, "leftred");                   //connect to Digital port 1

        LEDleft2Green = hardwareMap.get(DigitalChannel.class, "left2green");               //connect to Digital port 2
        LEDleft2Red = hardwareMap.get(DigitalChannel.class, "left2red");                   //connect to Digital port 3

        LEDrightRed = hardwareMap.get(DigitalChannel.class, "rightgreen");             //connect to Digital port 4
        LEDrightGreen = hardwareMap.get(DigitalChannel.class, "rightred");             //connect to Digital port 5

        LEDright2Green = hardwareMap.get(DigitalChannel.class, "right2green");             //connect to Digital port 6
        LEDright2Red = hardwareMap.get(DigitalChannel.class, "right2red");                 //connect to Digital port 7

//        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
        ledStick = hardwareMap.get(SparkFunLEDStick.class,"leds");


//        neopixels = hardwareMap.get(AdafruitNeoDriver.class,"neopixels");
//        neopixels.setNumberOfPixels(NUM_PIXELS);
    }
    public void start() {
        ledStick.setColor(1, Color.WHITE);
        ledStick.setBrightness(1,1);
        ledStick.setBrightness(2,0);
        ledStick.setBrightness(3,0);
        ledStick.setBrightness(4,0);
        ledStick.setBrightness(5,0);
        ledStick.setBrightness(6,0);
        ledStick.setBrightness(7,0);
        ledStick.setBrightness(8,0);
        ledStick.setBrightness(9,0);
        ledStick.setBrightness(10,0);
    }
    public void stop() {
        ledStick.setColor(1, Color.WHITE);
        ledStick.setBrightness(1,1);
        ledStick.setBrightness(2,0);
        ledStick.setBrightness(3,0);
        ledStick.setBrightness(4,0);
        ledStick.setBrightness(5,0);
        ledStick.setBrightness(6,0);
        ledStick.setBrightness(7,0);
        ledStick.setBrightness(8,0);
        ledStick.setBrightness(9,0);
        ledStick.setBrightness(10,0);
    }
    public void LEDinitReady() {
        LEDrightGreen.setMode(DigitalChannel.Mode.OUTPUT);
        LEDrightRed.setMode(DigitalChannel.Mode.INPUT);
        LEDrightGreen.setState(true);
        LEDrightRed.setState(false);

        LEDright2Green.setMode(DigitalChannel.Mode.OUTPUT);
        LEDright2Red.setMode(DigitalChannel.Mode.OUTPUT);
        LEDright2Green.setState(true);
        LEDright2Red.setState(false);

        LEDleft2Green.setMode(DigitalChannel.Mode.OUTPUT);
        LEDleft2Red.setMode(DigitalChannel.Mode.OUTPUT);
        LEDleft2Green.setState(true);
        LEDleft2Red.setState(false);

        LEDleftGreen.setMode(DigitalChannel.Mode.OUTPUT);
        LEDleftGreen.setState(true);
        LEDleftRed.setState(false);


//        LEDleftGreen.setMode(DigitalChannel.Mode.INPUT);
//        LEDleftRed.setMode(DigitalChannel.Mode.INPUT);
//        LEDleftGreen.setState(false);
//        LEDleftRed.setState(false);

    }
    public void LEDinitError() {
        LEDrightGreen.setMode(DigitalChannel.Mode.INPUT);
        LEDrightRed.setMode(DigitalChannel.Mode.OUTPUT);
        LEDrightGreen.setState(false);
        LEDrightRed.setState(true);

        LEDright2Green.setMode(DigitalChannel.Mode.OUTPUT);
        LEDright2Red.setMode(DigitalChannel.Mode.OUTPUT);
        LEDright2Green.setState(false);
        LEDright2Red.setState(true);

//        LEDleft2Green.setMode(DigitalChannel.Mode.INPUT);
//        LEDleft2Red.setMode(DigitalChannel.Mode.INPUT);
//        LEDleft2Green.setState(false);
//        LEDleft2Red.setState(false);

        LEDleft2Green.setMode(DigitalChannel.Mode.INPUT);
        LEDleft2Red.setMode(DigitalChannel.Mode.OUTPUT);
        LEDleft2Green.setState(false);
        LEDleft2Red.setState(true);

        LEDleftGreen.setMode(DigitalChannel.Mode.OUTPUT);
        LEDleftRed.setMode(DigitalChannel.Mode.OUTPUT);
        LEDleftGreen.setState(false);
        LEDleftRed.setState(true);
    }

    public void LED_QWIICinit() {


    }
}