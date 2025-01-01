package Diagnostic;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Disabled
public class Test_IMU extends TestItem {
    private IMU imu;
    double onValue;
    double offValue;

    public Test_IMU(String description, IMU imu) {
        super(description);
        this.imu = imu;

    }

    @Override
    public void run(boolean A_on, boolean Y_on, Telemetry telemetryA) {

        if ((A_on) || (Y_on)) {
            //;
        }
        telemetryA.addData("inu angle (degrees)= ", imu.getRobotYawPitchRollAngles().getPitch());

    }
}