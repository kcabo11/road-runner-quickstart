package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.QuinticSpline;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.path.Path;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;

@Config
@Autonomous(group = "Blue Side Auto")
public class RedMainAuto extends LinearOpMode {
    static SampleMecanumDrive robot;
    LibraryOpenCV opencv;
//    static LibraryTranslatePos translatePos;
    String RingConfig;

    static Pose2d startingpos;
    static Pose2d myPos;

    static Trajectory targetZoneDelivery;
    static Trajectory grabSecondWG;
    static Vector2d[] targetzonepath  = new Vector2d[3];
    static Vector2d grabWobbleGoal;

    public static void generatePlotPoints() {
         startingpos = new Pose2d(84,8);
         targetzonepath[0] = new Vector2d (84,0);//-60
         targetzonepath[1] = new Vector2d (108,0);//-60
         targetzonepath[2] = new Vector2d (132,0);//-60

        grabWobbleGoal = new Vector2d(12,24);


    }

        public void generatePath(String pos){
        int n = 0;
        DriveConstraints constraints = new DriveConstraints(//DriveConstraints.)
                80.0, 40.0, 0.0, Math.toRadians(180.0), Math.toRadians(180.0), 0.0);
        generatePlotPoints();

        DriveConstraints newconstraint = new DriveConstraints(40,20,0,
                Math.toRadians(180),Math.toRadians(180), 0);

            if (pos == "NONE"){

                n = 0;
            }
            else if (pos == "ONE"){
                n = 1;
            }
            else if (pos == "FOUR"){
                n = 2;
            }

            targetZoneDelivery = new TrajectoryBuilder(startingpos, newconstraint)
                    .lineTo(targetzonepath[n])
                    .build();

            grabSecondWG = new TrajectoryBuilder(targetZoneDelivery.end(), newconstraint)
                    .splineTo(grabWobbleGoal,180)
                    .build();
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
            telemetry.addData("timer", getRuntime());
            telemetry.update();
        }
        opencv.shutDownOpenCV();

        telemetry.addData("RunOpMode","");
        telemetry.update();

        generatePath(RingConfig);

        if (RingConfig == "NONE"){
            telemetry.addData("Position A","");
            telemetry.update();

            robot.followTrajectory(targetZoneDelivery);
            sleep(5000);
            robot.followTrajectory(grabSecondWG);

        }if (RingConfig == "ONE"){
            telemetry.addData("Position B", "");
            telemetry.update();

            robot.followTrajectory(targetZoneDelivery);
            sleep(5000);
            robot.followTrajectory(grabSecondWG);

        }if (RingConfig == "FOUR"){
            telemetry.addData("Position C", "");
            telemetry.update();

            robot.followTrajectory(targetZoneDelivery);
            sleep(5000);
            robot.followTrajectory(grabSecondWG);
        }
    }
}