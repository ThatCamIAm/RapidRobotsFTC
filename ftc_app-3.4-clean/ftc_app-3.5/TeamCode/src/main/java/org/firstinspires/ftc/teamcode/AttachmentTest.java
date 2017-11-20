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
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(gamepad1.dpad_up){
            robot.liftMotor.setPower(-0.5);

        }
        else if(gamepad1.dpad_down){
            robot.liftMotor.setPower(0);

        }
        else{
            robot.liftMotor.setPower(-0.3);
        }
        telemetry.addData("Lift Motor Power:", robot.liftMotor.getPower());

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