package org.firstinspires.ftc.teamcode.drive.opmode.Automous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import Sample.SampleMecanumDrive;

//@Disabled
@Config
@Autonomous(group = "Blue Side Auto")
public class RedMainAuto extends LinearOpMode {
    static SampleMecanumDrive robot;
//    LibraryOpenCV opencv;
//    static LibraryTranslatePos translatePos;
    String RingConfig;

    private DcMotor arm;
    private DcMotor arm2;
    private Servo wobble_grabber;

    static Pose2d startingpos;
    static Pose2d myPos;
    static Pose2d myPose;

    static Trajectory targetZoneDelivery;
    static Trajectory grabSecondWG;
    static Trajectory myTrajectory;
//    static Vector2d[] targetzonepath  = new Vector2d[3];
    static Vector2d grabWobbleGoal;

    public static void generatePlotPoints() {

//         targetzonepath[0] = new Vector2d (84,0);//-60
//         targetzonepath[1] = new Vector2d (108,0);//-60
//         targetzonepath[2] = new Vector2d (132,0);//-60

        grabWobbleGoal = new Vector2d(12,24);
        Pose2d myPose = new Pose2d(10, -5, Math.toRadians(90));


    }

        public void generatePath(String pos){

        generatePlotPoints();
        int n = 0;

            if (pos == "NONE"){

                n = 0;
            }
            else if (pos == "ONE"){
                n = 1;
            }
            else if (pos == "FOUR"){
                n = 2;
            }

//            Trajectory targetZoneDelivery = new TrajectoryBuilder()
//                    .lineTo(targetzonepath[n])
//                    .build();
//
//            grabSecondWG = new TrajectoryBuilder(targetZoneDelivery.end(),)
//                    .splineTo(grabWobbleGoal,180)
//                    .build();

            myTrajectory = robot.trajectoryBuilder(new Pose2d(0, 0, 0))
//                    .splineTo(new Pose2d(15, 15, 0))
                    .lineTo(new Vector2d(120, 0))
                    .build();

//            Trajectory myTrajectory = robot.trajectoryBuilder(new Pose2d(1,0,0)).strafeRight(10).forward(5).build();

}
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap);

        arm = hardwareMap.dcMotor.get("arm");
        arm2 = hardwareMap.dcMotor.get("arm2");
        wobble_grabber = hardwareMap.servo.get("wobble_grabber");
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();

//        opencv = new LibraryOpenCV(robot, telemetry, hardwareMap);
//        opencv.initOpenCV();
        telemetry.addData("OpenCV initialized", "");
        telemetry.update();

//        while (!isStarted()) {
////            RingConfig = opencv.findRingConfig();
//            telemetry.addData("timer", getRuntime());
//            telemetry.update();
//        }
//        opencv.shutDownOpenCV();
        waitForStart();

        telemetry.addData("RunOpMode","");
        telemetry.update();

        generatePath(RingConfig);

        robot.followTrajectory(myTrajectory);
        clawBack(robot);

//        if (RingConfig == "NONE"){
//            telemetry.addData("Position A","");
//            telemetry.update();
//
////            robot.followTrajectory(targetZoneDelivery);
////            sleep(5000);
////            robot.followTrajectory(grabSecondWG);
//
//        }if (RingConfig == "ONE"){
//            telemetry.addData("Position B", "");
//            telemetry.update();
//
////            robot.followTrajectory(targetZoneDelivery);
////            sleep(5000);
////            robot.followTrajectory(grabSecondWG);
//
//        }if (RingConfig == "FOUR"){
//            telemetry.addData("Position C", "");
//            telemetry.update();
//
////            robot.followTrajectory(targetZoneDelivery);
////            sleep(5000);
////            robot.followTrajectory(grabSecondWG);
//        }
    }

    public void clawBack (SampleMecanumDrive robot){
        arm.setPower(1);
        arm2.setPower(1);
        sleep(500);
        arm.setPower(0);
        arm2.setPower(0);
        sleep(250);
        wobble_grabber.setPosition(0);
    }
}