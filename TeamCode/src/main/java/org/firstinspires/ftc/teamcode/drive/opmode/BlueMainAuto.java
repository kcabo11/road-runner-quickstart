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
public class BlueMainAuto extends LinearOpMode {
    static SampleMecanumDrive drive;
    public ElapsedTime runtime = new ElapsedTime();
    LibraryOpenCV opencv;
    String SkystonePosition = "";

    static Pose2d startingpos;
    static Pose2d myPos;

    static Trajectory deliveringTargetZone;
    static Vector2d[] targetzonepath  = new Vector2d[3];


    public static void generatePlotPoints() {
         startingpos = new Pose2d(-60,-60);
         targetzonepath[0] = new Vector2d (0,-60);
         targetzonepath[1] = new Vector2d (0,60);
         targetzonepath[2] = new Vector2d (0,-60);

    }

        public void generatePath(String pos){
        int n = 0;
        DriveConstraints constraints = new DriveConstraints(//DriveConstraints.)
                80.0, 40.0, 0.0, Math.toRadians(180.0), Math.toRadians(180.0), 0.0);
        generatePlotPoints();

        DriveConstraints newconstraint = new DriveConstraints(40,20,0,
                Math.toRadians(180),Math.toRadians(180), 0);

            if (pos == "left"){

                n = 0;
            }
            else if (pos == "middle"){
                n = 1;
            }
            else if (pos == "right"){
                n = 2;
            }

            deliveringTargetZone = new TrajectoryBuilder(startingpos, newconstraint)
                    .lineTo(targetzonepath[n])
                    .build();
}
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
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

        String pos = "left";

        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();

        generatePath(pos);

        /* IF ringconfig 0 Rings

        drive forward (3.5,.5)
        strafe left(1.5,2.5)
        Run Shooter to hit power shots
        Drive forward (1.5,3.5)
         */

        drive.followTrajectory(deliveringTargetZone);

        /* IF ringconfig 1 Rings

        drive forward ()3.5,4.5
        strafe back (1.5,2.5)
        Run shooter to hit power shots
        drive forward (1.5,3.5)
         */

        /* IF ringconfig 4 rings
        drive forward (3.5,5.5)
        strafe back (1.5,2.5)
        run shooter to hit power shots
        drive forward (1.5,3.5)

         */

        }
    }
