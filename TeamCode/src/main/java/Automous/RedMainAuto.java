package Automous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.QuinticSpline;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.opmode.LibraryOpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    /** NO Ring Trajectory */
    static Trajectory grabSecondWGNONE;
    static Trajectory grabbingWGNONE;
    static Trajectory myTrajectoryNONE;
    static Trajectory toShootRingsNONE;
    static Trajectory beforeDeliveryNONE;
    static Trajectory secondDeliveryNONE;
    static Trajectory shootToGoalNONE;
    static Trajectory turningToShootNONE;
    static Trajectory beforeLine;
    static Trajectory parkNONE;
    /** ONE Ring Trajectory */
    static Trajectory grabSecondWGONE;
    static Trajectory grabbingWGONE;
    static Trajectory myTrajectoryONE;
    static Trajectory toShootRingsONE;
    static Trajectory beforeDeliveryONE;
    static Trajectory secondDeliveryONE;
    static Trajectory shootToGoalONE;
    static Trajectory turningToShootONE;
    static Trajectory parkONE;
    /** FOUR Ring Trajectory */
    static Trajectory grabSecondWGFOUR;
    static Trajectory grabbingWGFOUR;
    static Trajectory myTrajectoryFOUR;
    static Trajectory toShootRingsFOUR;
    static Trajectory beforeDeliveryFOUR;
    static Trajectory secondDeliveryFOUR;
    static Trajectory shootToGoalFOUR;
    static Trajectory turningToShootFOUR;
    static Trajectory parkFOUR;

    static QuinticSpline test;
    //    static Vector2d[] targetzonepath  = new Vector2d[3];
    /** No Ring Vector */
    static Vector2d grabWobbleGoalNONE;
    static Vector2d shootRingsNONE;
    static Vector2d turningNONE;
    static Vector2d clawGrabWobbleNONE;
    static Vector2d secondPlacementNONE;
    static Vector2d wobbleDropNONE;
    static Vector2d toRightNONE;
    static Vector2d ringPosNONE;
    static Vector2d turnToShootNONE;
    static Vector2d onLineNONE;
    static Vector2d toPark;
    static Vector2d tuningToGrabNONE;

    /** ONE Ring Vector */
    static Vector2d grabWobbleGoalONE;
    static Vector2d shootRingsONE;
    static Vector2d turningONE;
    static Vector2d clawGrabWobbleONE;
    static Vector2d secondPlacementONE;
    static Vector2d wobbleDropONE;
    static Vector2d toRightONE;
    static Vector2d ringPosONE;
    static Vector2d turnToShootONE;
    static Vector2d onLineONE;
    static Vector2d tuningToGrabONE;

    /** FOUR Ring Vector */
    static Vector2d grabWobbleGoalFOUR;
    static Vector2d shootRingsFOUR;
    static Vector2d turningFOUR;
    static Vector2d clawGrabWobbleFOUR;
    static Vector2d secondPlacementFOUR;
    static Vector2d wobbleDropFOUR;
    static Vector2d toRightFOUR;
    static Vector2d ringPosFOUR;
    static Vector2d turnToShootFOUR;
    static Vector2d onLineFOUR;
    static Vector2d tuningToGrabFOUR;

    public static void generatePlotPointsNone() {

//         targetzonepath[0] = new Vector2d (59, 0);//-60
//         targetzonepath[1] = new Vector2d (84,24);//-60
//         targetzonepath[2] = new Vector2d (107,0);//-60

        /** NO ring Config*/
        wobbleDropNONE = new Vector2d(67, -6);
        grabWobbleGoalNONE = new Vector2d(30, 14);
        clawGrabWobbleNONE = new Vector2d(22, 32);
        toRightNONE = new Vector2d(12, -2);
        secondPlacementNONE = new Vector2d(60, -3);
        ringPosNONE = new Vector2d(55, 35);
        shootRingsNONE = (new Vector2d(55, 36));

        toPark = new Vector2d(55, 30);
        onLineNONE = new Vector2d(76 , 30);
        turnToShootNONE = (new Vector2d(58, 33));
        turningNONE = (new Vector2d(60, -6));
    }
        public static void generatePlotPointsOne() {
            /** ONE Config */
        wobbleDropONE = new Vector2d(84, 24);
        grabWobbleGoalONE = new Vector2d(30, 10);
        clawGrabWobbleONE = new Vector2d(21, 30);
        toRightONE = new Vector2d(36, 54);
        secondPlacementONE = new Vector2d(99, 33);
        ringPosONE = new Vector2d(75, 35);
        shootRingsONE = (new Vector2d(75, 36));

        turningONE = (new Vector2d(60, 48));
        }
        public static void generatePlotPointsFour() {
        /** FOUR Config */
        wobbleDropFOUR = new Vector2d(110, -3);
//        shootRingsFOUR = (new Vector2d(60, 40));
//        turningFOUR = (new Vector2d(61, 48));
        grabWobbleGoalFOUR = new Vector2d(27, 12);
        clawGrabWobbleFOUR = new Vector2d(24, 35);
//        tuningToGrabFOUR = new Vector2d(25, 36);
        toRightFOUR = new Vector2d(12, -2);
//        toRightFOUR = new Vector2d(36, 54);
        secondPlacementFOUR = new Vector2d(115, -2);
        shootRingsFOUR = (new Vector2d(75, 35));
        ringPosFOUR = (new Vector2d(75,36));
        turnToShootFOUR = (new Vector2d(55, 35));

        onLineFOUR = new Vector2d(72, -2);


    }

    public void generatePathNone(String pos) {

        generatePlotPointsNone();
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
        /** 0 Ring Not Working */
        myTrajectoryNONE = robot.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineTo(wobbleDropNONE)
                .build();
        toShootRingsNONE = robot.trajectoryBuilder(myTrajectoryNONE.end()) //Pose2d(84,24))
                .splineToConstantHeading(grabWobbleGoalNONE, 0)
                .build();
        grabSecondWGNONE = robot.trajectoryBuilder(toShootRingsNONE.end()) //Pose2d(84,24))
                .splineTo(clawGrabWobbleNONE, Math.toRadians(108))
                .build();
        beforeDeliveryNONE = robot.trajectoryBuilder(grabSecondWGNONE.end())
                .lineTo(toRightNONE)
                .build();
        secondDeliveryNONE = robot.trajectoryBuilder(beforeDeliveryNONE.end())
                .splineTo(secondPlacementNONE,0)
                .build();
        shootToGoalNONE = robot.trajectoryBuilder(secondDeliveryNONE.end())
                .lineTo(ringPosNONE)
                .build();
        turningToShootNONE = robot.trajectoryBuilder(shootToGoalNONE.end())
                .splineTo(shootRingsNONE, Math.toRadians(160))
                .build();
//        beforeLine = robot.trajectoryBuilder(turningToShootNONE.end())
//                .lineTo(toPark)
//                .build();
        parkNONE = robot.trajectoryBuilder(turningToShootNONE.end())
                .back(30)
                .build();
    }
    public void generatePathOne(String pos) {
        generatePlotPointsOne();
        /** 1 Ring Not Working */
        myTrajectoryONE = robot.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineTo(wobbleDropONE)
                .build();
        toShootRingsONE = robot.trajectoryBuilder(myTrajectoryONE.end()) //Pose2d(84,24))
                .splineToConstantHeading(grabWobbleGoalONE, 0)
                .build();
        grabSecondWGONE = robot.trajectoryBuilder(toShootRingsONE.end()) //Pose2d(84,24))
                .splineTo(clawGrabWobbleONE, Math.toRadians(98))
                .build();
        beforeDeliveryONE = robot.trajectoryBuilder(grabSecondWGONE.end())
                .lineTo(toRightONE)
                .build();
        secondDeliveryONE = robot.trajectoryBuilder(beforeDeliveryONE.end())
                .splineTo(secondPlacementONE, 0)
                .build();
        shootToGoalONE = robot.trajectoryBuilder(secondDeliveryONE.end())
                .lineTo(ringPosONE)
                .build();
        turningToShootONE = robot.trajectoryBuilder(shootToGoalONE.end())
                .splineTo(shootRingsONE, Math.toRadians(150))
                .build();
        parkONE = robot.trajectoryBuilder(turningToShootONE.end())
                .back(10)
                .build();
    }
    public void generatePathFour(String pos) {
        generatePlotPointsFour();
        /** 4 Ring Configuration */
        myTrajectoryFOUR = robot.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineTo(wobbleDropFOUR)
                .build();
        toShootRingsFOUR = robot.trajectoryBuilder(myTrajectoryFOUR.end())
                .splineToConstantHeading(grabWobbleGoalFOUR, 0)
                .build();
        grabSecondWGFOUR = robot.trajectoryBuilder(toShootRingsFOUR.end()) //Pose2d(84,24))
                .splineTo(clawGrabWobbleFOUR, Math.toRadians(108))//116.62))
                .build();
        beforeDeliveryFOUR = robot.trajectoryBuilder(grabSecondWGFOUR.end())
                .lineTo(toRightFOUR)
                .build();
        secondDeliveryFOUR = robot.trajectoryBuilder(beforeDeliveryFOUR.end())
                .splineTo(secondPlacementFOUR, 0)
                .build();
//        shootToGoalFOUR = robot.trajectoryBuilder(secondDeliveryFOUR.end())
//                .lineTo(ringPosFOUR)
//                .build();
//        turningToShootFOUR = robot.trajectoryBuilder(shootToGoalFOUR.end())
//                .splineTo(shootRingsFOUR, Math.toRadians(160))
//                .build();
        parkFOUR = robot.trajectoryBuilder(secondDeliveryFOUR.end())
                .back(36)
                .build();
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

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        if (RingConfig == "NONE"){
            generatePathNone(RingConfig);
            telemetry.addData("Finished calculating Traj","");
            telemetry.update();
        }
        else if (RingConfig == "ONE"){
            generatePathOne(RingConfig);
            telemetry.addData("Finished calculating Traj","");
            telemetry.update();
        }
        else if (RingConfig == "FOUR"){
            generatePathFour(RingConfig);
            telemetry.addData("Finished calculating Traj","");
            telemetry.update();
        }
        wobble_grabber.setPosition(-1);
        waitForStart();

        if (RingConfig == "NONE"){
            robot.followTrajectory(myTrajectoryNONE);
            clawBack(robot);
            robot.followTrajectory(toShootRingsNONE);
            clawGrabWobble(robot);
            robot.followTrajectory(grabSecondWGNONE);
//        robot.followTrajectory(grabbingWG);
            liftWobble(robot);
            robot.followTrajectory(beforeDeliveryNONE);
            robot.followTrajectory(secondDeliveryNONE);
            justClawDownOpen(robot);
            robot.followTrajectory(shootToGoalNONE);
            robot.followTrajectory(turningToShootNONE);
            launchRings(robot);
            robot.followTrajectory(parkNONE);

        }
        else if (RingConfig == "ONE"){
            robot.followTrajectory(myTrajectoryONE);
            clawBack(robot);
            robot.followTrajectory(toShootRingsONE);
            clawGrabWobble(robot);
            robot.followTrajectory(grabSecondWGONE);
            liftWobble(robot);
            robot.followTrajectory(beforeDeliveryONE);
            robot.followTrajectory(secondDeliveryONE);
            justClawDownOpen(robot);
            robot.followTrajectory(shootToGoalONE);
            robot.followTrajectory(turningToShootONE);
            launchRings(robot);
            robot.followTrajectory(parkONE);
        }
        else if (RingConfig == "FOUR"){
             robot.followTrajectory(myTrajectoryFOUR);
             clawBack(robot);
             robot.followTrajectory(toShootRingsFOUR);
             clawGrabWobble(robot);
             robot.followTrajectory(grabSecondWGFOUR);
//             robot.followTrajectory(grabbingWGFOUR);
             liftWobble(robot);
             robot.followTrajectory(beforeDeliveryFOUR);
//            robot.followTrajectory(shootToGoalFOUR);
//            robot.followTrajectory(turningToShootFOUR);
             robot.followTrajectory(secondDeliveryFOUR);
             justClawDownOpen(robot);
             robot.followTrajectory(parkFOUR);
        }
    }

//    private void clawBack(SampleMecanumDrive robot) {
//        arm.setPower(-.8);
//        arm2.setPower(-.8);
//        sleep(400);
//        arm.setPower(0);
//        arm2.setPower(0);
//        sleep(300);
//        wobble_grabber.setPosition(1);
//        sleep(250);
//        arm.setPower(.8);
//        arm2.setPower(.8);
//        sleep(300);
//        arm.setPower(0);
//        arm2.setPower(0);
//        wobble_grabber.setPosition(0);
//        arm.setPower(.8);
//        arm2.setPower(.8);
//        sleep(100);
//        arm.setPower(0);
//        arm2.setPower(0);
//    }
    private void clawBack(SampleMecanumDrive robot) {
//        arm.setPower(-.7);
//        arm2.setPower(-.7);
//        sleep(500);
//        arm.setPower(0);
//        arm2.setPower(0);
//        sleep(300);
        while (arm.getCurrentPosition() < 1269){
            arm.setTargetPosition(1269);
            arm.setPower(-.8);
            arm2.setPower(-.8);
        }
        arm.setPower(0);
        arm2.setPower(0);
        sleep(300);
        wobble_grabber.setPosition(1);
        sleep(300);
//        sleep(250);
//        arm.setPower(.8);
//        arm2.setPower(.8);
//        sleep(300);
//        arm.setPower(0);
//        arm2.setPower(0);
        while (arm.getCurrentPosition() > 516){
            arm.setTargetPosition(516);
            arm.setPower(.8);
            arm2.setPower(.8);
        }
        arm.setPower(0);
        arm2.setPower(0);
        wobble_grabber.setPosition(0);
        sleep(200);
        while (arm.getCurrentPosition() > 0){
            arm.setTargetPosition(0);
            arm.setPower(.8);
            arm2.setPower(.8);
        }
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
    private void clawGrabWobble(SampleMecanumDrive robot){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (arm.getCurrentPosition() < 815){
            arm.setTargetPosition(815);
            arm.setPower(-.5);
            arm2.setPower(-.5);
        }
        arm.setPower(0);
        arm2.setPower(0);
        sleep(300);
        wobble_grabber.setPosition(1);

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
        sleep(400);
        launcherTime.reset();
        while (launcherTime.seconds() <= 3) {
            fireSelector.setPosition(0);
            sleep(500);
            fireSelector.setPosition(1);
            sleep(500);
        }
    }
}