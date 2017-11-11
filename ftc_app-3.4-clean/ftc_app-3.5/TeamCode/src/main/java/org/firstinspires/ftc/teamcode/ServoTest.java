package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by RoboticsAcc on 10/9/2017.
 */
@Autonomous(name = "ServoTestNew", group = "Tests")
public class ServoTest extends LinearOpMode {
    RobotHardware robot=new RobotHardware();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,telemetry);
        waitForStart();
        robot.servo2.scaleRange(0.0,1.0);
        robot.servo2.setPosition(0);
        sleep(3000);
        robot.servo2.setPosition(1);
        sleep(1000);
    }
}
