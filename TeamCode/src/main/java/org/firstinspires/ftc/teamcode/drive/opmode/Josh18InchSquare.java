package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Square drive Josh")
public class Josh18InchSquare extends LinearOpMode {


    DcMotor rightFront = null;
    DcMotor leftFront = null;
    DcMotor rightBack = null;
    DcMotor leftBack = null;
    public BNO055IMU imuActual = null;


    final static double DRIVE_SPEED = 0.3;
    final static double STOP_SPEED = 0;


    @Override
    public void runOpMode() {


        imuActual = hardwareMap.get(BNO055IMU.class, "imu_actual");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Sets parameters for gyro
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // initializes the gyro in the Rev IMU
        imuActual.initialize(parameters);



        rightFront = hardwareMap.dcMotor.get("right_front");
        rightBack = hardwareMap.dcMotor.get("right_back");
        leftFront = hardwareMap.dcMotor.get("left_front");
        leftBack = hardwareMap.dcMotor.get("left_back");

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        /**
         GO FORWARD
         */

        //reset encoders
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //set target position
        rightFront.setTargetPosition(-450);
        leftFront.setTargetPosition(450);
        rightBack.setTargetPosition(-450);
        leftBack.setTargetPosition(450);

        //set to RUN_TO_POSITION mode
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        rightBack.setPower(DRIVE_SPEED);
        rightFront.setPower(DRIVE_SPEED);
        leftBack.setPower(DRIVE_SPEED);
        leftFront.setPower(DRIVE_SPEED);



        while(rightFront.isBusy() && leftFront.isBusy() && rightBack.isBusy() && leftBack.isBusy()) {

            //wait until target pos is reached
        }



        rightBack.setPower(STOP_SPEED);
        rightFront.setPower(STOP_SPEED);
        leftBack.setPower(STOP_SPEED);
        leftFront.setPower(STOP_SPEED);

        sleep(1000);

        /**
         * STRAIFING TO ONE SIDE
         */
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //set target position
        rightFront.setTargetPosition(-900);
        leftFront.setTargetPosition(-900);
        rightBack.setTargetPosition(900);
        leftBack.setTargetPosition(900);

        //set to RUN_TO_POSITION mode
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        rightBack.setPower(DRIVE_SPEED);
        rightFront.setPower(DRIVE_SPEED);
        leftBack.setPower(DRIVE_SPEED);
        leftFront.setPower(DRIVE_SPEED);



        while(rightFront.isBusy() && leftFront.isBusy() && rightBack.isBusy() && leftBack.isBusy()) {

            //wait until target pos is reached
        }



        rightBack.setPower(STOP_SPEED);
        rightFront.setPower(STOP_SPEED);
        leftBack.setPower(STOP_SPEED);
        leftFront.setPower(STOP_SPEED);

        sleep(1000);

        /**
         * moving backwards
         */

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //set target position
        rightFront.setTargetPosition(400);
        leftFront.setTargetPosition(-400);
        rightBack.setTargetPosition(400);
        leftBack.setTargetPosition(-400);

        //set to RUN_TO_POSITION mode
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        rightBack.setPower(DRIVE_SPEED);
        rightFront.setPower(DRIVE_SPEED);
        leftBack.setPower(DRIVE_SPEED);
        leftFront.setPower(DRIVE_SPEED);



        while(rightFront.isBusy() && leftFront.isBusy() && rightBack.isBusy() && leftBack.isBusy()) {

            //wait until target pos is reached
        }



        rightBack.setPower(STOP_SPEED);
        rightFront.setPower(STOP_SPEED);
        leftBack.setPower(STOP_SPEED);
        leftFront.setPower(STOP_SPEED);


        sleep(1000);


        /**s
         * traifing the other waY
         */
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //set target position
        rightFront.setTargetPosition(900);
        leftFront.setTargetPosition(900);
        rightBack.setTargetPosition(-900);
        leftBack.setTargetPosition(-900);

        //set to RUN_TO_POSITION mode
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        rightBack.setPower(DRIVE_SPEED);
        rightFront.setPower(DRIVE_SPEED);
        leftBack.setPower(DRIVE_SPEED);
        leftFront.setPower(DRIVE_SPEED);



        while(rightFront.isBusy() && leftFront.isBusy() && rightBack.isBusy() && leftBack.isBusy()) {

            //wait until target pos is reached
        }



        rightBack.setPower(STOP_SPEED);
        rightFront.setPower(STOP_SPEED);
        leftBack.setPower(STOP_SPEED);
        leftFront.setPower(STOP_SPEED);


        sleep(1000);
    }
}