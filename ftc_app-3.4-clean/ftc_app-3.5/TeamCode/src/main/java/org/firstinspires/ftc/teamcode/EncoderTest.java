package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "EncoderTest", group="Tests")
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
        robot.servo2.setPosition(.9);
        sleep(1000);
        robot.turnDegrees(-90);
        robot.servo2.setPosition(.5);
        sleep(1000);
        robot.servo2.setPosition(.9);
        sleep(1000);
        robot.turnDegrees(90);
        /*robot.driveForwardInches(6, .3);
            sleep(500);
            robot.driveForwardInches(6, .3);
            sleep(500);
            robot.driveBackwardInches(6, .3);
            sleep(500);
            robot.turnDegrees(90);
            robot.driveBackwardInches(6, .3);
            sleep(500);
            robot.driveForwardInches(12, 0.75);
            sleep(500);
            robot.turnDegrees(180);
            robot.driveBackwardInches(12, 0.75);
        *///robot.setDrivePower(0.5,0.5);
        //sleep(1000);
        //sleep(3000);
        // goes to certain position
        //robot.driveForwardInches(36, .3);
        //sleep(3000);
        //robot.turnDegreesWithEncoders(90);
        //telemetry.addData("% Of Total Distance", (robot.frontLeftMotor.getCurrentPosition()*100.0)/robot.frontLeftMotor.getTargetPosition() + "%");
        //the sign of set position will determine direction
        //robot.resetEncoderValues();
        //super.stop();
      // robot.resetMotorsAndEncoders();

    }
}

