package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by RoboticsAcc on 10/9/2017.
 */
@Autonomous(name = "ServoTest`")
public class ServoTest extends LinearOpMode {
    Servo servo1;
    Servo servo2;

    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        waitForStart();
        servo2.setPosition(0);//-1 on 180 servo
        servo2.setPosition(0.5);//0 on 1    80 servo
        servo2.setPosition(1);//1 on 180 servo
        servo1.setPosition(0);
        sleep(3000);
        servo1.setPosition(0.5);
        sleep(3000);
        servo1.setPosition(0);
        telemetry.addData("Servo Position", servo1.getPosition());
        telemetry.update();
    }
}
