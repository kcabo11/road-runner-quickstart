package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "SampleOpModeJosh", group = "Tutorials")
public class SampleTeleOp extends LinearOpMode
{

    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor intake;
    private DcMotor flyWheel;
    private Servo fireSelector;
    private int fire_state = 0;

    @Override
    public void runOpMode() {


        rightFront = hardwareMap.dcMotor.get("right_front");
        leftFront = hardwareMap.dcMotor.get("left_front");
        rightBack = hardwareMap.dcMotor.get("right_back");
        leftBack = hardwareMap.dcMotor.get("left_back");
        intake = hardwareMap.dcMotor.get("intake");
        flyWheel = hardwareMap.dcMotor.get("fly_wheel");
        fireSelector = hardwareMap.servo.get("fire_selector");


        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

    while (opModeIsActive()) {

        //Strifing with the right stick
        rightFront.setPower(gamepad1.left_stick_x);
        leftFront.setPower(-gamepad1.left_stick_x);
        rightBack.setPower(-gamepad1.left_stick_x);
        leftBack.setPower(gamepad1.left_stick_x);


        //forward and backwards with right stick
        rightFront.setPower(gamepad1.right_stick_y);
        leftFront.setPower(gamepad1.right_stick_y);
        rightBack.setPower(gamepad1.right_stick_y);
        leftBack.setPower(gamepad1.right_stick_y);

        //TURNING
        if (gamepad1.right_stick_x >= .01) {
            rightFront.setPower(.8);
            leftFront.setPower(-.8);
            rightBack.setPower(.8);
            leftBack.setPower(-.8);
        }
        if (-gamepad1.right_stick_x >= .01) {
            rightFront.setPower(-.8);
            leftFront.setPower(.8);
            rightBack.setPower(-.8);
            leftBack.setPower(.8);
        }

        //intake forwards
        if (gamepad1.right_trigger > 0){
            intake.setPower(.8);
        }
        //intake backwards
        if (gamepad1.left_trigger > 0) {
            intake.setPower(-.8);
        }
        //flywheel
        if (gamepad1.right_bumper) {
            flyWheel.setPower(-1);
        }
        else {
            flyWheel.setPower(0);
        }


        //fire selector
        switch (fire_state) {
            case 0:
                if(gamepad1.a){
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


        telemetry.addData("H nutter", "yes");
        telemetry.addData("left_front_enc " , leftFront.getCurrentPosition());
        telemetry.addData("right_front_enc " , rightFront.getCurrentPosition());
        telemetry.addData("left_back_enc " , leftBack.getCurrentPosition());
        telemetry.addData("right_back_enc " , rightBack.getCurrentPosition());
        telemetry.addData("fire_state", fire_state);

        telemetry.update();
    }
    }
}
