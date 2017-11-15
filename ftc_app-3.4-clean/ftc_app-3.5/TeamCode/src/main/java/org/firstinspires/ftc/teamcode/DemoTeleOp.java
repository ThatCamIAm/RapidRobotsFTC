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
        robot.servo2.setPosition(.9);
        robot.servo3.setPosition(0);
        robot.servo4.setPosition(1);
    }

    private void processDriveMotors() {
        float throttle = gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;

        double rightPower = throttle - direction;
        double leftPower = direction + throttle;
        //restricting the values so they stay within -1 and 1
        leftPower = Range.clip(leftPower, -.7, .7);
        rightPower = Range.clip(rightPower, -.7, .7);

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
    //move to the top later
    private enum liftState{
        Init,
        Down,
        Falling,
        Lifting,
        Up;
    }
    public liftState CurrentLiftState = liftState.Init;
    public int armposition=0;
    public int lastarmposition = -1;
    public int armmax = 270;
    public int armmin = 0;
    private void liftMotorStateMachine(){
        liftState nextState;
        nextState = CurrentLiftState;
        switch (CurrentLiftState){
            case Init:
                armposition=0;
                nextState=liftState.Down;
                robot.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(0.3);
                break;
            case Up:
                if (gamepad1.dpad_down) {
                    nextState = liftState.Falling;
                    armposition = armposition - 10;
                }
                break;
            case Down:
                if (gamepad1.dpad_up) {
                    nextState = liftState.Lifting;
                    armposition = armposition + 10;
                }
                break;
            case Falling:
                if (armposition <= armmin){
                    nextState = liftState.Down;
                }
                else if (gamepad1.dpad_up) {
                    nextState = liftState.Lifting;
                    armposition = armposition + 10;
                }
                else if (gamepad1.dpad_down) {
                    nextState = liftState.Down.Falling;
                    armposition = armposition - 10;
                }
                break;
            case Lifting:
                if (armposition > armmax){
                    nextState = liftState.Up;
                }
                else if (gamepad1.dpad_up) {
                    nextState = liftState.Lifting;
                    armposition = armposition + 10;
                }
                else if (gamepad1.dpad_down) {
                    nextState = liftState.Down.Falling;
                    armposition = armposition - 10;
                }
                break;

        }
        //clamp armposition
        armposition =Math.max(armposition, armmin);
        armposition =Math.min(armposition, armmax);
        if (lastarmposition != armposition)
        {
            //set new position
            lastarmposition = armposition;
            robot.liftMotor.setTargetPosition(armposition);
        }
        telemetry.addData("Current LiftMotor state",CurrentLiftState);
        telemetry.addData("Next LiftMotor state",nextState);
        telemetry.addData("Arm position",armposition);
        telemetry.addData("Last Arm position",lastarmposition);
        telemetry.addData("Current motor position",robot.liftMotor.getCurrentPosition());
        telemetry.update();
        CurrentLiftState=nextState;
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
        //liftMotorControl();
        liftMotorStateMachine();
    }

    public void stop() {
            robot.reset();
        }

}