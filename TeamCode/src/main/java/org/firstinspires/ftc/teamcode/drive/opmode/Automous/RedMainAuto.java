package org.firstinspires.ftc.teamcode.drive.opmode.Automous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.QuinticSpline;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.LibraryOpenCV;

import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import Sample.SampleMecanumDrive;

//@Disabled
@Config
@Autonomous(group = "Blue Side Auto")
public class RedMainAuto extends LinearOpMode {
    static SampleMecanumDrive robot;
    LibraryOpenCV opencv;
    String RingConfig;
    public ElapsedTime launcherTime = new ElapsedTime();

    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor intake;
    private DcMotor flyWheel;
    private Servo fireSelector;
    private Servo intake_aid;
    private DcMotor arm;
    private DcMotor arm2;
    private Servo wobble_grabber;

    static Trajectory targetZoneDelivery;
    static Trajectory grabSecondWG;
    static Trajectory grabbingWG;
    static Trajectory myTrajectory;
    static Trajectory toShootRings;
    static Trajectory beforeDelivery;
    static Trajectory secondDelivery;
    static Trajectory shootToGoal;
    static Trajectory park;
    static QuinticSpline test;
    //    static Vector2d[] targetzonepath  = new Vector2d[3];
    static Vector2d grabWobbleGoal;
    static Vector2d shootRings;
    static Vector2d turning;
    static Vector2d clawGrabWobble;
    static Vector2d secondPlacement;
    static Vector2d wobbleDrop;
    static Vector2d toRight;
    static Vector2d onLine;
    static Vector2d tuningToGrab;
    public static void generatePlotPoints() {

//         targetzonepath[0] = new Vector2d (59, 0);//-60
//         targetzonepath[1] = new Vector2d (84,24);//-60
//         targetzonepath[2] = new Vector2d (107,0);//-60

        wobbleDrop = new Vector2d(84, 24);
        grabWobbleGoal = new Vector2d(30, 14);
        clawGrabWobble = new Vector2d(24, 32);
        toRight = new Vector2d(36, 54);
        secondPlacement = new Vector2d(96, 35);
        onLine = new Vector2d(72, -2);

        shootRings = (new Vector2d(60, 48));
        turning = (new Vector2d(61, 48));

        /** 4 Ring Configuration
        shootRings = (new Vector2d(60, 48));
        turning = (new Vector2d(61, 48));
        wobbleDrop = new Vector2d(107, -3);
        grabWobbleGoal = new Vector2d(30, 16);
        clawGrabWobble = new Vector2d(24, 35);
        tuningToGrab = new Vector2d(25, 36);
        toRight = new Vector2d(12, 0);
        secondPlacement = new Vector2d(109, -2);
        onLine = new Vector2d(72, -2);
         **/

    }

    public void generatePath(String pos) {

        generatePlotPoints();
        int n = 0;

        if (pos == "NONE") {

            n = 0;
        } else if (pos == "ONE") {
            n = 1;
        } else if (pos == "FOUR") {
            n = 2;
        }

//            Trajectory targetZoneDelivery = new TrajectoryBuilder()
//                    .lineTo(targetzonepath[n])
//                    .build();
        myTrajectory = robot.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineTo(wobbleDrop)
                .build();
//        toShootRings = robot.trajectoryBuilder(new Pose2d(84, 24,0))
//                .lineTo(toRight)
//                .build();
        toShootRings = robot.trajectoryBuilder(new Pose2d(84, 24, 0)) //Pose2d(84,24))
                .splineToConstantHeading(grabWobbleGoal, 0)
                .build();
        grabSecondWG = robot.trajectoryBuilder(new Pose2d(30, 14, 0)) //Pose2d(84,24))
                .splineTo(clawGrabWobble, 90)
                .build();
//        grabbingWG = robot.trajectoryBuilder(new Pose2d(24, 35, 90))
//                .lineToConstantHeading(tuningToGrab)
//                .build();
        beforeDelivery = robot.trajectoryBuilder(new Pose2d(24, 35,90))
                .lineTo(toRight)
                .build();
        secondDelivery = robot.trajectoryBuilder(new Pose2d(36,48, 90))
                .splineTo(secondPlacement, 0)
                .build();
        park = robot.trajectoryBuilder(new Pose2d(84,24, 0))
                .back(10)
                .build();

        /** 4 Ring Configuration
        myTrajectory = robot.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineTo(wobbleDrop)
                .build();
        toShootRings = robot.trajectoryBuilder(new Pose2d(107, 0, 0)) //Pose2d(84,24))
                .splineToConstantHeading(grabWobbleGoal, 0)
                .build();
        grabSecondWG = robot.trajectoryBuilder(new Pose2d(30, 16, 0)) //Pose2d(84,24))
                .splineTo(clawGrabWobble, 90)
                .build();
        grabbingWG = robot.trajectoryBuilder(new Pose2d(24, 35, 90))
                .lineToConstantHeading(tuningToGrab)
                .build();
        beforeDelivery = robot.trajectoryBuilder(new Pose2d(24, 35,90))
                .lineTo(toRight)
                .build();
        secondDelivery = robot.trajectoryBuilder(new Pose2d(12, 0, 90))
                .splineTo(secondPlacement, 0)
                .build();
        park = robot.trajectoryBuilder(new Pose2d(109, -2, 0))
                .back(30)
                .build();
         **/
//        test = new QuinticSpline(new QuinticSpline(0,0,0,0));


//            Trajectory myTrajectory = robot.trajectoryBuilder(new Pose2d(1,0,0)).strafeRight(10).forward(5).build();

    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new SampleMecanumDrive(hardwareMap);

        rightFront = hardwareMap.dcMotor.get("right_front");
        leftFront = hardwareMap.dcMotor.get("left_front");
        rightBack = hardwareMap.dcMotor.get("right_back");
        leftBack = hardwareMap.dcMotor.get("left_back");
        arm = hardwareMap.dcMotor.get("arm");
        arm2 = hardwareMap.dcMotor.get("arm2");
        wobble_grabber = hardwareMap.servo.get("wobble_grabber");
        intake = hardwareMap.dcMotor.get("intake");
        flyWheel = hardwareMap.dcMotor.get("fly_wheel");
        fireSelector = hardwareMap.servo.get("fire_selector");
        intake_aid = hardwareMap.servo.get("intake_aid");
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();

        opencv = new LibraryOpenCV(robot, telemetry, hardwareMap);
        opencv.initOpenCV();
        telemetry.addData("OpenCV initialized", "");
        telemetry.update();

        while (!isStarted()) {
            RingConfig = opencv.findRingConfig();
            telemetry.addData("opencv reading", opencv.findRingConfig());
            telemetry.addData("timer", getRuntime());
            telemetry.update();
        }
        opencv.shutDownOpenCV();
        wobble_grabber.setPosition(-1);
        waitForStart();

        telemetry.addData("RunOpMode", "");
        telemetry.update();

        generatePath(RingConfig);

        robot.followTrajectory(myTrajectory);
        clawBack(robot);
        robot.followTrajectory(toShootRings);
        clawGrabWobble(robot);
        robot.followTrajectory(grabSecondWG);
//        robot.followTrajectory(grabbingWG);
        liftWobble(robot);
        robot.followTrajectory(beforeDelivery);
        robot.followTrajectory(secondDelivery);
        justClawDownOpen(robot);
        robot.followTrajectory(park);

        /** 4 Ring Configuration
        robot.followTrajectory(myTrajectory);
        clawBack(robot);
        robot.followTrajectory(toShootRings);
        clawGrabWobble(robot);
        robot.followTrajectory(grabSecondWG);
        robot.followTrajectory(grabbingWG);
        liftWobble(robot);
        robot.followTrajectory(beforeDelivery);
        robot.followTrajectory(secondDelivery);
        justClawDownOpen(robot);
        robot.followTrajectory(park);
         **/
    }

    private void clawBack(SampleMecanumDrive robot) {
        arm.setPower(-.8);
        arm2.setPower(-.8);
        sleep(300);
        arm.setPower(0);
        arm2.setPower(0);
        sleep(250);
        wobble_grabber.setPosition(1);
        sleep(200);
        arm.setPower(.8);
        arm2.setPower(.8);
        sleep(400);
        arm.setPower(0);
        arm2.setPower(0);
    }
    private void justClawDownOpen(SampleMecanumDrive robot){
        wobble_grabber.setPosition(1);
        sleep(200);
        arm.setPower(.8);
        arm2.setPower(.8);
        sleep(400);
        arm.setPower(0);
        arm2.setPower(0);
    }
//    private void justClawUp

    private void clawGrabWobble(SampleMecanumDrive robot){
        arm.setPower(-.4);
        arm2.setPower(-.4);
        sleep(600);
        arm.setPower(0);
        arm2.setPower(0);

    }
    private void liftWobble(SampleMecanumDrive robot){
        wobble_grabber.setPosition(-1);
//        sleep(300);
//        arm.setPower(.5);
//        arm2.setPower(.5);
//        sleep(500);
//        arm.setPower(0);
//        arm2.setPower(0);
    }

    private void launchRings(SampleMecanumDrive robot) {
        flyWheel.setPower(-1);
        launcherTime.reset();
        while (launcherTime.seconds() <= 4) {
            fireSelector.setPosition(1);
            sleep(500);
            fireSelector.setPosition(-1);
            sleep(500);
        }
    }
}