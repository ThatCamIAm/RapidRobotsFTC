package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Abhishek Vangipuram on 12/7/2017.
 */
@Autonomous(name = "ServoTestAuton",group = "Tests")
public class ServoTestAuton extends LinearOpMode {
    RobotHardware robot=new RobotHardware();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,telemetry);
        robot.servo2.setPosition(0);
        sleep(300);
        robot.servo2.getController().pwmDisable();
        robot.setDrivePower(-0.2,-0.2);
        sleep(1000);
        robot.resetMotors();
        robot.servo2.getController().pwmEnable();
        robot.servo2.setPosition(0.6);
        sleep(500);
        robot.resetMotors();
    }
}
