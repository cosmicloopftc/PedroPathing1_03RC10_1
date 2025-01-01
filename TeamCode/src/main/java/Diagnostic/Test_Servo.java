package Diagnostic;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class Test_Servo extends TestItem {
    private Servo servo;
    double onValue;
    double offValue;

    public Test_Servo(String description, Servo servo, double offValue, double onValue) {
        super(description);
        this.servo = servo;
        this.offValue = offValue;           //min allowed servo position
        this.onValue = onValue;             //max allowed servo position
    }

    @Override
    public void run(boolean A_on, boolean Y_on, Telemetry telemetryA) {
        double presentServoPosition = servo.getPosition();
        if ((A_on) || (Y_on)) {
            servo.setPosition(onValue);
        } else {
            servo.setPosition(offValue);
        }
        telemetryA.addData("Current Servo position = ", "%.3f", servo.getPosition());
    }
}