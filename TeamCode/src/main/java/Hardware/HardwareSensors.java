package Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import android.graphics.Color;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure
/*Revision
 *12/27/2024: modified from FIRST external example--color sensor to detect RGB and Hue
 *
 *
 *
 */



public class HardwareSensors {
//    public ColorSensor colorTest;
//    public int detected_color;
//    float hsvValues[] = {0F,0F,0F};
//    final float values[] = hsvValues;

    public DistanceSensor distanceTest;
    /** The colorSensor field will contain a reference to our color sensor hardware object */
    public NormalizedColorSensor colorTest;
    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;


    /*Constructor*/
    public HardwareSensors() {
    }

    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap) {
        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // use NormalizedColorSensor over ColorSensor, because NormalizedColorSensor consistently
        // gives values between 0 and 1, while values you get from ColorSensor are dependent on specific sensor you're using.
        colorTest = hardwareMap.get(NormalizedColorSensor.class, "colorTest");
        distanceTest = hardwareMap.get(DistanceSensor.class, "distanceTest");

        try {
            runSample(); // actually execute the sample
        } finally {
            // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
            // as pure white, but it's too much work to dig out what actually was used, and this is good
            // enough to at least make the screen reasonable again.
            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }


//        //Save reference to Hardware map
//        colorIntake1 = hardwareMap.get(ColorSensor.class,"colorTest");
//        detected_color = Color.HSVToColor(0xff, values);

    }

    public void runSample() {
        // If possible, turn light on at beginning (it might already be on anyway, we just make sure it is if we can).
        if (colorTest instanceof SwitchableLight) {
            ((SwitchableLight) colorTest).enableLight(true);

        }

    }





    public void start() {
    }

    public void stop () {
    }




    public float getColorInfo(int index) {
        final float[] hsvValues = new float[3];
        //take the readings, etc.
        NormalizedRGBA colors = ((NormalizedColorSensor) colorTest).getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
//        Color.RGBToHSV((int) (colorIntake1.red() * 8),
//                (int) (colorIntake1.green() * 8),
//                (int) (colorIntake1.blue() * 8),
//                hsvValues);
        //return colorIntake1.red();
        //return hsvValues[index];
        return 1;
    }

    public double getDistance(){
        return distanceTest.getDistance(DistanceUnit.CM);
    }


}