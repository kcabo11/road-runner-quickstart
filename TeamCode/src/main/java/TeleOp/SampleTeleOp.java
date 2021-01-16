package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;


@TeleOp(name = "SampleOpModeJosh", group = "Tutorials")
public class   SampleTeleOp extends LinearOpMode
{

    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor intake;
    private DcMotor flyWheel;
    private Servo fireSelector;
    private DcMotor arm;
    private DcMotor arm2;
    private Servo wobble_grabber;
    public ElapsedTime clawruntime = new ElapsedTime();
    private int fire_state = 0;
    private int claw_state = 0;

    String drivingState = "";

    private int direction = -1;
    // Setting scaling to full speed.
    private double scaleFactor = 1;
    private double scaleTurningSpeed = 1;

    private int wobble_pos = 1;

    private double theta = 22.5;
    private double delta = 45;
    private double speed = 0;
    private double stick_directon = 0;


    private void setPowers (double leftFrontSpd, double leftBackSpd, double rightFrontSpd, double rightBackSpd) {
        leftFront.setPower(leftFrontSpd);
        leftBack.setPower(leftBackSpd);
        rightFront.setPower(rightFrontSpd);
        rightBack.setPower(rightBackSpd);
}

    @Override
    public void runOpMode() {
        rightFront = hardwareMap.dcMotor.get("right_front");
        leftFront = hardwareMap.dcMotor.get("left_front");
        rightBack = hardwareMap.dcMotor.get("right_back");
        leftBack = hardwareMap.dcMotor.get("left_back");
        intake = hardwareMap.dcMotor.get("intake");
        flyWheel = hardwareMap.dcMotor.get("fly_wheel");
        fireSelector = hardwareMap.servo.get("fire_selector");
        arm = hardwareMap.dcMotor.get("arm");
        arm2 = hardwareMap.dcMotor.get("arm2");
        wobble_grabber = hardwareMap.servo.get("wobble_grabber");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

    while (opModeIsActive()) {


        drivingState = "";

        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;
// When the direction value is reversed this if statement inverts the addition and subtraction for turning.
// Default mode: The robot starts with the scaleTurningSpeed set to 1, scaleFactor set to 1, and direction set to forward.
        if (direction == 1) {
            final double v1 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v2 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v3 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v4 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftBack.setPower(v3);
            rightBack.setPower(v4);
        } else {
            final double v1 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v2 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v3 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v4 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftBack.setPower(v3);
            rightBack.setPower(v4);
        }
//        if (gamepad1.right_stick_x == 0) {
//            speed = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(-gamepad1.left_stick_y, 2));
//            stick_directon = Math.toDegrees(Math.asin(-gamepad1.left_stick_y / speed));
//
//            if (gamepad1.left_stick_x < 0 && -gamepad1.left_stick_y > 0) {
//                stick_directon = 180 - Math.abs(stick_directon);
//            } else if (gamepad1.left_stick_x < 0 && -gamepad1.left_stick_y < 0) {
//                stick_directon = 180 + Math.abs(stick_directon);
//            } else if (gamepad1.left_stick_x > 0 && -gamepad1.left_stick_y < 0) {
//                stick_directon = 360 - Math.abs(stick_directon);
//            } else {
//                stick_directon = stick_directon;
//            }
//
//            //1 direction, +x, +y 1st  /
//            //if ((stick_directon <= (theta + delta)) && (stick_directon >= (theta + .01))) { //  22.51 < sd < 67.5;
//            if ((stick_directon > 22.5) && (stick_directon <= 67.5)) {
//                leftFront.setPower(speed);
//                leftBack.setPower(0);  //for the diagonal upward 45 degree
//                rightFront.setPower(0);
//                rightBack.setPower(speed);
//                drivingState = "1st region";
//            }
//
//            // 2 position +y |
//            //else if ((stick_directon <= (theta + (delta * 2))) && (stick_directon >= (theta + delta + .01))) {
//            else if ((stick_directon > 67.5) && (stick_directon <= 112.5)) {
//                leftFront.setPower(speed);
//                leftBack.setPower(speed);
//                rightFront.setPower(speed);
//                rightBack.setPower(speed);
//                drivingState = "2nd Region";
//            }
//
//            //3 position -x, +y  \
//            else if ((stick_directon > 112.5) && (stick_directon <= 157.5)) {
//                leftFront.setPower(0);
//                leftBack.setPower(speed);
//                rightFront.setPower(speed);
//                rightBack.setPower(0);
//                drivingState = "3rd Region";
//            }
//            //4 position -x (straifing)  <––
//            else if ((stick_directon > 157.5) && (stick_directon <= 202.5)) {
//                leftFront.setPower(-speed);
//                leftBack.setPower(speed);
//                rightFront.setPower(speed);
//                rightBack.setPower(-speed);
//                drivingState = "4th Region";
//            }
//            //5 position -y, -x   /
//            else if ((stick_directon > 202.5) && (stick_directon <= 247.5)) {
//                leftFront.setPower(-speed);
//                leftBack.setPower(0);
//                rightFront.setPower(0);
//                rightBack.setPower(-speed);
//                drivingState = "5th Region";
//            }
//            // 6 position -y  |
//            else if ((stick_directon > 247.5) && (stick_directon <= 292.5)) {
//                leftFront.setPower(-speed);
//                leftBack.setPower(-speed);
//                rightFront.setPower(-speed);
//                rightBack.setPower(-speed);
//                drivingState = "6th Region";
//            }
//            //7 position +x, -y   \
//            else if ((stick_directon > 292.5) && (stick_directon <= 337.5)) {
//                leftFront.setPower(0);
//                leftBack.setPower(-speed);
//                rightFront.setPower(-speed);
//                rightBack.setPower(0);
//                drivingState = "7th Region";
//            }
//            //8 position ––> (strafing)
//            else if ((stick_directon > 337.5) && (stick_directon <= 22.5)) {
//                leftFront.setPower(speed);
//                leftBack.setPower(-speed);
//                rightFront.setPower(-speed);
//                rightBack.setPower(speed);
//                drivingState = "8th Region";
//            }
//          else { setPowers(0, 0, 0, 0);}
            //Turning
//            if (gamepad1.right_stick_x > 0) {
//                rightFront.setPower(.4);
//                leftFront.setPower(-.4);
//                rightBack.setPower(.4);
//                leftBack.setPower(-.4);
//            } else if (-gamepad1.right_stick_x > 0) {
//                rightFront.setPower(-.4);
//                leftFront.setPower(.4);
//                rightBack.setPower(-.4);
//                leftBack.setPower(.4);
//            }
//            else {
//                leftBack.setPower(0);
//                leftFront.setPower(0);
//                rightBack.setPower(0);
//                rightFront.setPower(0);
//            }
//        }

        if (gamepad2.right_trigger > 0){   //Intake Forwards
            intake.setPower(.8);
        } else if (gamepad2.left_trigger > 0){    //Intake backwards
           intake.setPower(-.8);
        } else {
            intake.setPower(0);
        }


        //flywheel
        if (gamepad2.right_bumper) {
            flyWheel.setPower(-1);
        }
        else if (gamepad2.left_bumper) {
            flyWheel.setPower(.3);
        } else {
            flyWheel.setPower(0);
        }


        //fire selector
        switch (fire_state) {
            case 0:
                if(gamepad2.a){
                    fireSelector.setPosition(0);
                    //manual case mover thing

                    sleep (200);
                    fire_state = 1;
                    break;
                }
            case 1:
                if (fireSelector.getPosition() == 0) {
                    fireSelector.setPosition(1);
                    sleep(200);
                    fire_state ++;
                    break;
                }
            case 2:
                if (fireSelector.getPosition() == 1) {
                    fire_state = 0;
                    break;
                }
        }


        //Arm mover motor
        if (gamepad2.dpad_up) {
            arm.setPower(.3);
            arm2.setPower(.3);
        } else if (gamepad2.dpad_down){
            arm.setPower(-.3);
            arm2.setPower(-.3);
        } else {
            arm.setPower(0);
            arm2.setPower(0);
        }

        switch (claw_state) {
            case (0):
                if (gamepad2.x) {
                    claw_state = 1;
                    wobble_grabber.setPosition(wobble_pos);
                }
                break;
            case (1):
                clawruntime.reset();
                if (!gamepad2.x) {
                    wobble_grabber.setPosition(-1);
                    claw_state = 0;
                    if (wobble_pos == -1) {
                        claw_state = 0;
                    } else {
                        wobble_pos = 1;
                    }
                }
                break;
        }


        /**
         GAMEPAD2
         */


        //I commented this out because I think there is problems with the motors/servos when they are used in two controllers


//        if (gamepad2.right_trigger > 0){   //Intake Forwards
//            intake.setPower(.8);
//        } else if (gamepad2.left_trigger > 0){    //Intake backwards
//            intake.setPower(-.8);
//        } else {
//            intake.setPower(0);
//        }
//
//
//        //flywheel
//        if (gamepad2.right_bumper) {
//            flyWheel.setPower(-1);
//        }
//        else {
//            flyWheel.setPower(0);
//        }
//
//
//        //fire selector
//        switch (fire_state) {
//            case 0:
//                if(gamepad2.a){
//                    fireSelector.setPosition(0);
//                    //manual case mover thing
//
//                    sleep (200);
//                    fire_state = 1;
//                    break;
//                }
//            case 1:
//                if (fireSelector.getPosition() == 0) {
//                    fireSelector.setPosition(1);
//                    sleep(200);
//                    fire_state ++;
//                    break;
//                }
//            case 2:
//                if (fireSelector.getPosition() == 1) {
//                    fire_state = 0;
//                    break;
//                }
//        }


        telemetry.addData("H nutter", "yes");
        telemetry.addData("left_front_enc " , leftFront.getCurrentPosition());
        telemetry.addData("right_front_enc " , rightFront.getCurrentPosition());
        telemetry.addData("left_back_enc " , leftBack.getCurrentPosition());
        telemetry.addData("right_back_enc " , rightBack.getCurrentPosition());
        telemetry.addData("fire_state", fire_state);
        telemetry.addData("claw_state", claw_state);
        telemetry.addData("right back power", rightBack.getPower());
        telemetry.addData("right Front power", rightFront.getPower());
        telemetry.addData("left back power", leftBack.getPower());
        telemetry.addData("left front power", leftFront.getPower());
        telemetry.addData("claw", wobble_grabber.getPosition());
        telemetry.addData("gp1 right stick y", gamepad1.right_stick_y);
        telemetry.addData("gp1 right stick x", gamepad1.right_stick_x);
        telemetry.addData("gp1 left stick y", gamepad1.left_stick_y);
        telemetry.addData("gp1 left stick x", gamepad1.left_stick_x);
        telemetry.addData("driving State", drivingState);
        telemetry.addData("computed spd", speed);
        telemetry.addData("stick_direction", stick_directon);

        telemetry.update();
    }
    }
}
