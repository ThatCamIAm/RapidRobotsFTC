package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Abhishek Vangipuram on 11/22/2017.
 */
@Autonomous(name = "MotorTest2",group = "Tests")
public class MotorTest2 extends LinearOpMode {
    private RobotHardware robot = new RobotHardware();
    MotorTest drivemethod = new MotorTest();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        drivemethod.driveForwardInchesWithTime(15);
        sleep(2000);
        drivemethod.driveForwardInchesWithTime(-15);
        sleep(2000);
    }

}