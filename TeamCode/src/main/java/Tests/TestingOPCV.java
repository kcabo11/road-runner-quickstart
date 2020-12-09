package Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Sample.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.LibraryOpenCV;
@Disabled
@Config
@Autonomous(group = "Blue Side Auto")
public class TestingOPCV extends LinearOpMode {
    static SampleMecanumDrive robot;
    LibraryOpenCV opencv;

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
            opencv.findRingConfig();
            telemetry.addData("timer", getRuntime());
            telemetry.update();
        }
        opencv.shutDownOpenCV();

}}