package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.HardwareBeep;

/**
 * @author Beep Patrol
 * <p>
 * <b>Summary:</b>
 * <p>
 * This is our main teleOp program which controls the robot during the driver controlled period.
 */
@Disabled
@TeleOp(name = "TeleOp Program", group = "TankDrive")
public class TeleOpProgram extends OpMode {

    // Calling hardware map.
    private HardwareBeep robot;

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
    }

    @Override
    public void loop() {

    }
}
