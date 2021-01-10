package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "StraifingTest", group = "Tests")
public class StraifingTest extends LinearOpMode
{

    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;





    @Override
    public void runOpMode() {


        rightFront = hardwareMap.dcMotor.get("right_front");
        leftFront = hardwareMap.dcMotor.get("left_front");
        rightBack = hardwareMap.dcMotor.get("right_back");
        leftBack = hardwareMap.dcMotor.get("left_back");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            //straifing with right stick
            rightFront.setPower(gamepad1.right_stick_x);
            leftFront.setPower(-gamepad1.right_stick_x);
            rightBack.setPower(-gamepad1.right_stick_x);
            leftBack.setPower(gamepad1.right_stick_x);


            //forward and backwards with right stick
            rightFront.setPower(gamepad1.right_stick_y);
            leftFront.setPower(gamepad1.right_stick_y);
            rightBack.setPower(gamepad1.right_stick_y);
            leftBack.setPower(gamepad1.right_stick_y);

            idle();
        }

        telemetry.addData("right front", rightFront.getPower());
        telemetry.addData("left front", leftFront.getPower());
        telemetry.addData("right back", rightBack.getPower());
        telemetry.addData("left back", leftBack.getPower());

    }
}

