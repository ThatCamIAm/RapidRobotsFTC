package org.firstinspires.ftc.teamcode;

import android.graphics.LinearGradient;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Alden Seki on 10/28/2017.
 */

@TeleOp(name = "AldenTeleOp", group = "TeleOpStuff")
public class TeleOpSeki1 extends OpMode{
private RobotHardware robot = new RobotHardware();
    @Override
    public void init() {
     robot.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
     checkGamepad();
    }
    public void checkGamepad(){
        if(gamepad2.a){
        openClamp();
        }
        else if(gamepad2.b){
          closeClamp();

        }
         while(gamepad2.y){
          raiseClamp();
        }
        while(gamepad2.x){
            lowerClamp();
        }
    }
    public void openClamp(){
      robot.servo3.setPosition(0);
      robot.servo4.setPosition(1);
    }
    public void closeClamp(){
        robot.servo3.setPosition(0.5);
        robot.servo4.setPosition(0.5);
    }
    public void raiseClamp(){
        robot.liftMotor.setPower(0.1);

    }
    public void lowerClamp(){
        robot.liftMotor.setPower(-0.1);
    }

}



