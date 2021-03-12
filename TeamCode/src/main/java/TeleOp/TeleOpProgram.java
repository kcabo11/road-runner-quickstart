package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.teamcode.util.Encoder;


@TeleOp(name = "SampleOpModeJosh", group = "Tutorials")
public class TeleOpProgram extends LinearOpMode
{

    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor intake;
    private DcMotor flyWheel;
    private DcMotor arm;
    private DcMotor ramp_adjustor;
    private Servo intake_aid;
    private Servo wobble_grabber;
    private Servo fireSelector;
    public ElapsedTime clawruntime = new ElapsedTime();
    public ElapsedTime fireruntime = new ElapsedTime();
    public ElapsedTime intakehelperruntime = new ElapsedTime();
    private int fire_state = 0;
    private int claw_state = 0;
    private int intake_state = 0;
    private int rampSensor_state = 0;
    private int flywheel_state = 0;
    private double flywheelMultiplier = 1;

    String drivingState = "";

    private int direction = -1;
    // Setting scaling to full speed.
    private double scaleFactor = 1;
    private double scaleTurningSpeed = .6;

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
        intake_aid = hardwareMap.servo.get("intake_aid");
        arm = hardwareMap.dcMotor.get("arm");
        wobble_grabber = hardwareMap.servo.get("wobble_grabber");
        ramp_adjustor = hardwareMap.dcMotor.get("ramp_adjustor");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

    while (opModeIsActive()) {


        drivingState = "";

        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
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

       /**
       GMAEPAD 2:
        */

        if (gamepad2.right_trigger > 0){   //Intake Forwards
            intake.setPower(.8);
        } else if (gamepad2.left_trigger > 0){    //Intake backwards
           intake.setPower(-.8);
        } else {
            intake.setPower(0);
        }


        //flywheel
        if (gamepad2.right_bumper) {
            flyWheel.setPower(-1 * flywheelMultiplier);
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
                    fireruntime.reset();
                    fireSelector.setPosition(0);
                    fire_state = 1;
                    break;
                }
            case 1:
                if (fireruntime.milliseconds() >= 300) {
                    fireruntime.reset();
                    fireSelector.setPosition(1);
                    fire_state = 0;
                    break;
                }
        }

        //Arm mover motor
        if (gamepad2.dpad_up) {
            arm.setPower(-.3);
        } else if (gamepad2.dpad_down){
            arm.setPower(.3);
        } else {
            arm.setPower(0);
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

        //intake helper
        switch (intake_state) {
            case 0:
                if(gamepad2.b){
                    intakehelperruntime.reset();
                    intake_aid.setPosition(0);
                    intake_state = 1;
                    break;
                }
            case 1:
                if (intakehelperruntime.milliseconds() >= 300) {
                    intake_aid.setPosition(.6);
                    intake_state = 0;
                    break;
                }
        }

        if(gamepad2.left_stick_y > 0) {
            ramp_adjustor.setPower(.3);
        } else if(gamepad2.left_stick_y < 0) {
            ramp_adjustor.setPower(-.3);
        } else {
            ramp_adjustor.setPower(0);
        }


       // Flywheel power control
        switch (flywheel_state) {
            case 0:
                if(gamepad2.right_stick_button){
                    flywheelMultiplier = .85;
                    flywheel_state = 1;
                }
                break;
            case 1:
                if (!gamepad2.right_stick_button) {
                    flywheel_state = 2;
                }
                break;
            case 2:
                if (gamepad2.right_stick_button) {
                    flywheelMultiplier = 1;
                    flywheel_state = 3;
                }
                break;
            case 3:
                if(!gamepad2.right_stick_button) {
                    flywheel_state = 0;
                }
                break;
        }

//
//        switch (rampSensor_state) {
//            case 0:
//                if(gamepad2.x){
//                   while (ramp_sensor.getState() == false) {
//                       ramp_adjustor.setTargetPosition(-999999);
//                   }
//                   ramp_adjustor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    intake_state = 1;
//                    break;
//                }
//            case 1:
//                if (ramp) {
//
//                    intake_state = 0;
//                    break;
//                }
//        }




        telemetry.addData("H nutter", "yes");
//        telemetry.addData("left_front_enc " , leftFront.getCurrentPosition());
//        telemetry.addData("right_front_enc " , rightFront.getCurrentPosition());
//        telemetry.addData("left_back_enc " , leftBack.getCurrentPosition());
//        telemetry.addData("right_back_enc " , rightBack.getCurrentPosition());
//        telemetry.addData("fire_state", fire_state);
//        telemetry.addData("claw_state", claw_state);
//        telemetry.addData("intake_state", intake_state);
//        telemetry.addData("right back power", rightBack.getPower());
//        telemetry.addData("right Front power", rightFront.getPower());
//        telemetry.addData("left back power", leftBack.getPower());
//        telemetry.addData("left front power", leftFront.getPower());
//        telemetry.addData("claw", wobble_grabber.getPosition());
//        telemetry.addData("gp1 right stick y", gamepad1.right_stick_y);
//        telemetry.addData("gp1 right stick x", gamepad1.right_stick_x);
//        telemetry.addData("gp1 left stick y", gamepad1.left_stick_y);
//        telemetry.addData("gp1 left stick x", gamepad1.left_stick_x);
//        telemetry.addData("driving State", drivingState);
//        telemetry.addData("computed spd", speed);
//        telemetry.addData("stick_direction", stick_directon);
        telemetry.addData("ramp_motor_enc", ramp_adjustor.getCurrentPosition());
        telemetry.addData("arm_enc", arm.getCurrentPosition());
        telemetry.addData("fire_selector position", fireSelector.getPosition());
        telemetry.addData("ramp_adjustor", ramp_adjustor.getCurrentPosition());
        telemetry.addData("flywheel speed", flywheelMultiplier);
        telemetry.addData("flywheel_case", flywheel_state);
        telemetry.addData("right stick botton", gamepad2.right_stick_button);

        telemetry.update();
    }
    }
}

//192.168.43.1