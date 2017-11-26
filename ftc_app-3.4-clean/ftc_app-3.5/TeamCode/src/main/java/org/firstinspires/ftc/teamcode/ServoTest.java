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
    CRServo crservo;
    Servo servo;
    RobotHardware robot=new RobotHardware();
    double position = 0;
    @Override
    public void init() {
        robot.init(hardwareMap,telemetry);
        crservo=hardwareMap.crservo.get("crservo");
        crservo.close();
        robot.servo2.close();
    }

    @Override
    public void loop() {
        if(gamepad1.x||gamepad1.b)
            position=0.5;
        else if(gamepad1.y)
            position=0;
        else if(gamepad1.a)
            position=1;
        robot.servo2.setPosition(position);
    }

    @Override
    public void stop() {
    }
}
