package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by RoboticsAcc on 10/9/2017.
 */
@TeleOp(name = "ServoTestNew", group = "Tests")
public class ServoTest extends OpMode {
    Servo servo;
    double position = 0.5;
    @Override
    public void init() {servo=hardwareMap.servo.get("servo2");}

    @Override
    public void loop() {
            if(gamepad1.x||gamepad1.b)
                position = 0.5;
            else if(gamepad1.y)
                position = 1;
            else if(gamepad1.a)
                position = 0;

        servo.setPosition(position);
    }

    @Override
    public void stop() {
    }
}
