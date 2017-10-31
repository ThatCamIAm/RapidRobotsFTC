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
        robot.init(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Heading", robot.angles.firstAngle);
            telemetry.update();
        }

        /*robot.driveStraight(0.5,2);
        robot.turnDegrees(90);

        robot.resetMotors();
        robot.resetEncoderValues();*/
        }
    }

