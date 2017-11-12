package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Lil_Pump's_TeleOp")
public class DemoTeleOp extends OpMode {

    private RobotHardware robot = new RobotHardware();

    @Override
    public void init() {
            robot.init(hardwareMap, telemetry);
        }
    @Override
    public void start() {
        //robot.servo1.setPosition(0);
        robot.servo2.setPosition(.7);
        robot.servo3.setPosition(0);
        robot.servo4.setPosition(1);
    }

    private void processDriveMotors() {
        float throttle = gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;

        float rightPower = throttle - direction;
        float leftPower = direction + throttle;
        //restricting the values so they stay within -1 and 1
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        robot.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.setDrivePower(leftPower,rightPower);
    }
    private void servoControl(){
        //0 = 0 degrees     1 = 180 degrees  value = degree/180
        if (gamepad1.b){
            robot.servo3.setPosition(.8);
            robot.servo4.setPosition(.2);
        }
        else if(gamepad1.a){
            robot.servo3.setPosition(0.2);
            robot.servo4.setPosition(0.8);
        }

    }
    private void liftMotorControl(){
        double liftMotorPower;
        int counts=0;
        if(gamepad1.dpad_up){
            robot.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            if(robot.liftMotor.getCurrentPosition()<robot.ANDYMARK_REVOLUTION/4){
                robot.liftMotor.setTargetPosition(robot.liftMotor.getCurrentPosition()+50);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(0.3);
            }
        }
        if(gamepad1.dpad_down){
            if(robot.liftMotor.getCurrentPosition()>0){
                robot.liftMotor.setTargetPosition(robot.liftMotor.getCurrentPosition()-50);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(0.3);
            }
        }

    }
    @Override
    public void loop() {
        processDriveMotors();
        servoControl();
        liftMotorControl();
    }

    public void stop() {
            robot.reset();
        }

}