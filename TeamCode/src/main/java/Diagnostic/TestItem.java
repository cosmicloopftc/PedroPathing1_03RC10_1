package Diagnostic;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@Disabled
abstract public class TestItem {
    private Telemetry telemetryA;
    private String description;
    protected TestItem(String description) {
        this.description = description;
    }
    public String getDescription() {
        return description;
    }
    abstract public void run(boolean A_on, boolean Y_on, Telemetry telemetryA);

}