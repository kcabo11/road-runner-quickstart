package Tests;//package Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp (name = "BlinkinTest")
public class REVblinkinTest extends LinearOpMode{

   private RevBlinkinLedDriver lights;


   @Override
    public void runOpMode() {

       lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

       waitForStart();
       while (opModeIsActive()) {


           if (gamepad2.a) {
               lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
           }
       }
   }

}