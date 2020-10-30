package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * @author Beep Patrol
 * <p>
 * <b>Summary:</b>
 * <p>
 * This is our main teleOp program which controls the robot during the driver controlled period.
 */
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
