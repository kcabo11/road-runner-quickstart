package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(group = "Blue Side Skystone Auto")
public class TestingOpenCV extends LinearOpMode {
    static HardwareBeep robot;
    LibraryOpenCV opencv;
    String SkystonePosition;

    public String GenerateRingConfig (String pos) {
        int n = 0;

        if (pos == "NONE") {

            n = 0;
        } else if (pos == "ONE") {
            n = 1;
        } else if (pos == "FOUR") {
            n = 2;
        }
        return pos;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareBeep();
        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();

        while (!isStarted()) {
            SkystonePosition = opencv.findRingConfig();
            telemetry.addData("OpenCV initialized", "");
            telemetry.update();
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }
        opencv.shutDownOpenCV();

        GenerateRingConfig(SkystonePosition);
        telemetry.addData("Ring configuration", SkystonePosition);
        telemetry.update();
//        waitForStart();
//        opencv = new LibraryOpenCV(robot,telemetry, hardwareMap);
//        opencv.initOpenCV();
//        opencv.findRingConfig();
//        sleep(5000);
//        telemetry.addData("OpenCV initialized", "");
//        telemetry.update();
    }
}
