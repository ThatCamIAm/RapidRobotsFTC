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
        robot.driveStraight(0.5,6);
        telemetry.addData("Heading", robot.angles.firstAngle);
        telemetry.update();
        robot.turnDegrees(90);
        telemetry.addData("Heading",robot.angles.firstAngle);
        telemetry.update();
        robot.resetMotors();
        robot.resetEncoderValues();
        super.stop();
        }
    }

