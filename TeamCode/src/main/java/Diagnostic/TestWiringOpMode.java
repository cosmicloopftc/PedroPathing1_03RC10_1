package Diagnostic;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

/*THIS CLASS SETUP THE MENU.  IT PULLS EVERYTHING TOGETHER---MAIN PROGRAM FOR THE TESTING HARDWARE.
3/3/2024 update hardware and add Y and A input for 2 type of "on" button
(child class) TestProgrammingBoard + TestItem --> TestingMotor... --> TestWIRING_OpMode (parent superclass)
*/



@Config
@TeleOp (group="test", name= "TestWIRING DIAGNOSTIC_1.1")
public class TestWiringOpMode extends OpMode {
    TestHardware robot = new TestHardware();

    ArrayList<TestItem> tests;
    boolean wasDown, wasUp;
    int testNum;

    private Telemetry telemetryA;

    @Override
    public void init() {
        robot.init(hardwareMap);
        tests = robot.getTests();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the hardware diagnostic program.");
        telemetryA.update();
    }

    @Override
    public void loop() {
        // move up in the list of test
        if (gamepad1.dpad_up && !wasUp) {
            testNum--;
            if (testNum < 0) {
                testNum = tests.size() - 1;
            }
        }
        wasUp = gamepad1.dpad_up;

        // move down in the list of tests
        if (gamepad1.dpad_down && !wasDown) {
            testNum++;
            if (testNum >= tests.size()) {
                testNum = 0;
            }
        }
        wasDown = gamepad1.dpad_down;

//Put instructions on the telemetry
        telemetryA.addLine("Use Up/Down on D-pad for choices");
        //telemetry.addLine("Press A to run test");
        telemetryA.addLine("");
        telemetryA.addLine("For Servo, Press A for --, Y ++");
        telemetryA.addLine("For other hardware, Press A or Y to run test");
        telemetryA.addLine("");
        //put the test on the telemetry
        TestItem currTest = tests.get(testNum);

        telemetryA.addData("Test:", currTest.getDescription());
        //run or don’t run based on a
        telemetryA.addLine("");
        if(gamepad1.a && !gamepad1.y){
            gamepad1.a = true;
            gamepad1.y = false;
        }
        if(gamepad1.y && !gamepad1.a){
            gamepad1.y = true;
            gamepad1.a = false;
        }
        currTest.run(gamepad1.a, gamepad1.y, telemetryA);

        telemetryA.update();
    }
}