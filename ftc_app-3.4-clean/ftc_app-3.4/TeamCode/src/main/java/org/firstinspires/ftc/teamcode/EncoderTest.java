package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "EncoderTest")
/**
 * Created by Abhishek Vangipuram on 9/2/2017.
 */

public class EncoderTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        // goes to certain position
        robot.driveForwardInches(36, .6);
        //the sign of set position will determine direction
        while (robot.frontLeftMotor.isBusy()||robot.frontRightMotor.isBusy()||robot.backLeftMotor.isBusy()||robot.backRightMotor.isBusy()){
            telemetry.addData("% Of Total Distance", (robot.frontLeftMotor.getCurrentPosition()*100.0)/robot.frontLeftMotor.getTargetPosition() + "%");
            telemetry.update();
        }
        robot.resetEncoderValues();
        super.stop();
    }
}

