package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by RoboticsAcc on 11/12/2017.
 */
@TeleOp(name = "AttachmentTest", group = "Tests")
public class AttachmentTest extends OpMode{
    RobotHardware robot=new RobotHardware();

    @Override
    public void init() {
        robot.init(hardwareMap,telemetry);
    }

    @Override
    public void start() {
        robot.servo2.setPosition(.7);
        robot.servo3.setPosition(1);
        robot.servo4.setPosition(0);
    }
    private void servoControl(){
        //0 = 0 degrees     1 = 180 degrees  value = degree/180
        if (gamepad1.b){
            robot.closeGrabber();
        }
        else if(gamepad1.a){
            robot.openGrabber();
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
    int loops=0;
    @Override
    public void loop() {
        servoControl();
        liftMotorControl();
    }

    @Override
    public void stop() {
        robot.reset();
    }
}