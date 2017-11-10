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
        robot.resetMotorsAndEncoders();
        robot.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveForwardInches(24,.3);
        //sleep(3000);
        // goes to certain position
        //robot.driveForwardInches(36, .3);
        //sleep(3000);
        //robot.turnDegreesWithEncoders(90);
        //telemetry.addData("% Of Total Distance", (robot.frontLeftMotor.getCurrentPosition()*100.0)/robot.frontLeftMotor.getTargetPosition() + "%");
        //the sign of set position will determine direction
        //robot.resetEncoderValues();
        //super.stop();
        robot.resetMotorsAndEncoders();

    }
}

