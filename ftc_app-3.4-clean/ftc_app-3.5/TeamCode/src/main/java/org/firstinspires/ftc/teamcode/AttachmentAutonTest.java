package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by RoboticsAcc on 11/16/2017.
 */
@Autonomous(name="AttachmentAutonTest",group = "Tests")
public class AttachmentAutonTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,telemetry);
        waitForStart();
        robot.reset();
        /*robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.closeGrabber();
        robot.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.liftMotor.setTargetPosition(150);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(0.5);
        sleep(3000);
        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftMotor.setPower(0);
        sleep(5000);
        */
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.closeGrabber();
        robot.liftMotor.setPower(-.2);
        sleep(15000);
        robot.resetMotorsAndEncoders();
    }
}
