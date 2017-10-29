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
        while(opModeIsActive()){
            telemetry.addData("Runtime",robot.runtime.seconds());
            telemetry.addData("Heading", robot.angles.firstAngle);
            telemetry.addData("Left Motor Power", robot.frontLeftMotor.getPower());
            telemetry.addData("Right Motor Power",robot.frontRightMotor.getPower());
            telemetry.update();
        }

        /*robot.driveStraight(0.5);
        robot.turnDegrees(90);
        telemetry.addData("Heading",robot.angles.firstAngle);
        sleep(5000);
        telemetry.update();
        */
        robot.resetMotors();
        robot.resetEncoderValues();
        }
    }

