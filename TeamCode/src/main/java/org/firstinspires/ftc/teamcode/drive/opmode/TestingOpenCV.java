package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "Blue Side Skystone Auto")
public class TestingOpenCV extends LinearOpMode {
    static SampleMecanumDrive robot;
    LibraryOpenCV opencv;
    String RingConfig;

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
        robot = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();

        opencv = new LibraryOpenCV(robot, telemetry, hardwareMap);
        opencv.initOpenCV();
        telemetry.addData("OpenCV initialized", "");
        telemetry.update();

        while (!isStarted()) {
            RingConfig = opencv.findRingConfig();
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

        GenerateRingConfig(RingConfig);

        if (RingConfig == "NONE"){
            telemetry.addData("Ring Configuration", RingConfig);
            telemetry.update();
        }if (RingConfig == "ONE"){
            telemetry.addData("Ring Configuration", RingConfig);
            telemetry.update();
        }if (RingConfig == "FOUR"){
            telemetry.addData("Ring Configuration", RingConfig);
            telemetry.update();
        }
    }
}
