package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by RoboticsAcc on 10/9/2017.
 */
@TeleOp(name = "ServoTestNew", group = "Tests")
public class ServoTest extends OpMode {
    RobotHardware robot=new RobotHardware();
    double position = 0;
    @Override
    public void init() {
        robot.init(hardwareMap,telemetry);
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
