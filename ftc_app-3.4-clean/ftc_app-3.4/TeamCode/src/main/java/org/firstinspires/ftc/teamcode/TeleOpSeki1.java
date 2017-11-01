/*
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

*/
/**
 * Created by Alden Seki on 10/28/2017.
 *//*

@TeleOp(name = "AldenTeleOp", group = "TeleOpStuff")
public class TeleOpSeki1 extends OpMode
{
private DcMotor motorLeft;
private DcMotor motorRight;
private Servo armServo;
private static final double ARM_RETRACTED_POSITION = 0.2;
    private static final double ARM_EXTEND_POSITION = 0.8;
    @Override
    public void runOpMode() throws InterruptedException {
        motorLeft = hardwareMap.dcMotor.get("MotorLeft");
        motorRight = hardwareMap.dcMotor.get("MotorRight");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
       armServo = hardwareMap.servo.get("armServo");

        waitForStart();


        public void checkControllers(){
            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);
            if (gamepad2.a) {
                armServo.setPosition(0.8);


            }
            if (gamepad2.b) {
                armServo.setPosition(0.2);


            }
        }
        idle();
        }



        }


    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
*/

