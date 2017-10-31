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
        robot.init(hardwareMap, telemetry);
        waitForStart();
        // goes to certain position
        robot.driveForwardInches(36, .3);
        //telemetry.addData("% Of Total Distance", (robot.frontLeftMotor.getCurrentPosition()*100.0)/robot.frontLeftMotor.getTargetPosition() + "%");
        telemetry.addData("Current LeftMotor Counts", (robot.backLeftMotor.getCurrentPosition()));
        telemetry.addData("Left Target Pos",robot.backLeftMotor.getTargetPosition());
        telemetry.addData("Left Motor Power",robot.backLeftMotor.getPower());
        telemetry.addData("Current RightMotor Counts", (robot.backRightMotor.getCurrentPosition()));
        telemetry.addData("Right Target Pos",robot.backRightMotor.getTargetPosition());
        telemetry.addData("Right Motor Power",robot.backRightMotor.getPower());
        telemetry.update();
        //the sign of set position will determine direction
        robot.resetEncoderValues();
        super.stop();

    }
}

