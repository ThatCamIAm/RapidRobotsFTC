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
        robot.driveBackwardInches(10,0.2,10);
        sleep(1000);
        robot.driveForwardInches(10,0.2,10);
        sleep(1000);
        robot.resetMotorsAndEncoders();
    }
}

