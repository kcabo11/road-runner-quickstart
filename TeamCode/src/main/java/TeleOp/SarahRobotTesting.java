package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Sarahs Program", group = "Tutorials")
public class SarahRobotTesting extends LinearOpMode
{
    private int intake_state = 0;

    private DcMotor clawMover;
    private Servo grabber;


    @Override
    public void runOpMode() {


        clawMover = hardwareMap.dcMotor.get("Claw_Mover");
        grabber = hardwareMap.servo.get("Grabber");

        waitForStart();

        while (opModeIsActive()) {

            //STRAIFING WITH RIGHT STICK
           if(gamepad1.right_stick_x >= .01){
               clawMover.setPower(.5);

           }
            if(-gamepad1.right_stick_x >= .01){
                clawMover.setPower(-.5);

            }


            if (gamepad1.a) {
                grabber.setPosition(1);
            }
            else if (gamepad1.b){
                grabber.setPosition(0);
            }



            idle();
            telemetry.addData("H nutter", "yes");


        }
    }
}
