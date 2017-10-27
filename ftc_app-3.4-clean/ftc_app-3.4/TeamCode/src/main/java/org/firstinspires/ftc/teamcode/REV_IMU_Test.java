package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Created by Abhishek Vangipuram on 10/24/2017.
 */
@Autonomous (name="REV_IMU_Test")
public class REV_IMU_Test extends LinearOpMode {
    RobotHardware robot=new RobotHardware();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        //robot.driveStraight(0.3);
        //sleep(5000);
        robot.turnDegrees(90);
        robot.reset();
        super.stop();
        }
    }

