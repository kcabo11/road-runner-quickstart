package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @author Beep Patrol
 * <p>
 * <b>Summary:</b>
 * <p>
 * This is our main teleOp program which controls the robot during the driver controlled period.
 */
@TeleOp(name = "TelepArmProgram", group = "TankDrive")
public class TestingArmProgram extends OpMode {

    public DcMotor WobbleArm = null;
    public Servo claw = null;
    private int arm_state = 0;
    private ElapsedTime armtime = new ElapsedTime();

    @Override
    public void init() {
        WobbleArm = hardwareMap.get(DcMotor.class, "wobbleArm");
        claw = hardwareMap.get(Servo.class, "claw");

        telemetry.addData("Say", "Hello Driver");

        WobbleArm.setDirection(DcMotorSimple.Direction.FORWARD);
        WobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        if (gamepad1.right_stick_y < 0) {
            WobbleArm.setPower(-.5);
        } else if (gamepad1.right_stick_y > 0) {
            WobbleArm.setPower(.5);
        } else {
            WobbleArm.setPower(0);
        }

        if (gamepad1.x){
            claw.setPosition(1);
            telemetry.addData("xpressed","");
            telemetry.update();
        }
        if (gamepad1.y){
            claw.setPosition(-1);
            telemetry.addData("ypressed","");
            telemetry.update();
        }
        telemetry.addData("arm encoder", WobbleArm.getCurrentPosition());
        telemetry.update();
//        switch (arm_state) {
//            case 0:
//                WobbleArm.setPower(gamepad1.right_stick_y * .5);
//
//                if (gamepad1.left_bumper) {
//                    WobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    WobbleArm.setTargetPosition();
//                    WobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    WobbleArm.setPower(.5);
//                    arm_state = 1;
//                } else if (gamepad1.right_bumper) {
//                    WobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    WobbleArm.setTargetPosition(-);
//                    WobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    WobbleArm.setPower(.5);
//                    arm_state = 1;
//                }
//                break;
//            case 1:
//                armtime.reset();
//                if (WobbleArm.isBusy()){
//                    arm_state = 2;
//                }
//                break;
//            case 2:
//                if (!WobbleArm.isBusy() || armtime.seconds() >= 2){
//                    arm_state = 0;
//                    WobbleArm.setPower(0);
//                }
//                break;
//        }
    }
    public void stop(){

    }
}
