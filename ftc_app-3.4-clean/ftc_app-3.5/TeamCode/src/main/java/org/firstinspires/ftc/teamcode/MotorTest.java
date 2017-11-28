package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Abhishek Vangipuram on 11/22/2017.
 */
@Autonomous(name = "MotorTest",group = "Tests")
public class MotorTest extends LinearOpMode {
    private RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        robot.closeGrabber();
        sleep(500);
        robot.liftMotor.setPower(-0.5);
        sleep(300);
        robot.liftMotor.setPower(-0.3);
        driveForwardInchesWithTIme(18);
        sleep(1000);
        robot.liftMotor.setPower(-0.5);
        sleep(300);
        robot.liftMotor.setPower(-0.3);
        robot.turnDegrees(-90);
        robot.openGrabber();
        sleep(500);
        robot.liftMotor.setPower(0.2);
        sleep(500);
        robot.liftMotor.setPower(0);

    }
    public void driveForwardInchesWithTIme(double inches){
        final double SECONDS_PER_INCH = 0.16;
        double timeDouble=1000*(inches*SECONDS_PER_INCH);
        long timeLong= (long) timeDouble;
        if(inches>=0){
            robot.setDrivePower(0.1,0.1);
        }
        else{
            robot.setDrivePower(-0.1,-0.1);
        }
        sleep(timeLong);
        robot.resetMotors();
    }
}