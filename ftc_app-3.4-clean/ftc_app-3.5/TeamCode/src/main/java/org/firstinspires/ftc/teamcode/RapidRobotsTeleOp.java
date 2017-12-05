package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Lil_Pump's_TeleOp")
public class RapidRobotsTeleOp extends OpMode {
    private RobotHardware robot = new RobotHardware();
    float throttle, direction;
    boolean forward;
    boolean currentForwardButtonvalue, lastForwardButtonValue;
    double CRServoPower;
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
        public void start() {
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.servo1.setPosition(0.4);
        robot.servo2.setPosition(0.5);
        //setting drive as forward at start
        forward=true;

    }
    private void processDriveMotors() {

        throttle = -gamepad1.left_stick_y;
        direction = gamepad1.left_stick_x;

        double rightPower = throttle - direction;//-d+t
        double leftPower = direction + throttle;//d+t
        //restricting the values so they stay within -1 and 1
        if(gamepad1.right_bumper){
            leftPower = Range.clip(leftPower, -.1, .1);
            rightPower = Range.clip(rightPower, -.1, .1);

        }
        else{
            leftPower = Range.clip(leftPower, -.5, .5);
            rightPower = Range.clip(rightPower, -.5, .5);
        }
        //logic for switching forwards or backwards driving
        if(gamepad1.back){
            lastForwardButtonValue=currentForwardButtonvalue;
            currentForwardButtonvalue=true;
        }
        else{
            lastForwardButtonValue=currentForwardButtonvalue;
            currentForwardButtonvalue=false;
        }
        if(currentForwardButtonvalue&&!lastForwardButtonValue){
            forward=!forward;
        }
        if(forward){
            robot.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else{
            robot.frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        robot.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*if(gamepad1.left_bumper){
            robot.frontLeftMotor.setPower(.4);
            robot.backLeftMotor.setPower(.4);
            robot.frontRightMotor.setPower(-.4);
            robot.backRightMotor.setPower(-.4);

        }

        if(gamepad1.right_bumper){
            robot.frontLeftMotor.setPower(-.4);
            robot.backLeftMotor.setPower(-.4);
            robot.frontRightMotor.setPower(.4);
            robot.backRightMotor.setPower(.4);

        }*/

        robot.frontLeftMotor.setPower(leftPower);
        robot.backLeftMotor.setPower(leftPower);
        robot.frontRightMotor.setPower(rightPower);
        robot.backRightMotor.setPower(rightPower);
        telemetry.addData("BackLeftMotor Power:", robot.backLeftMotor.getPower());
        telemetry.addData("FrontLeftMotor Power:", robot.frontLeftMotor.getPower());
        telemetry.addData("BackRightMotor Power:", robot.backRightMotor.getPower());
        telemetry.addData("FrontRIghtMotor Power:", robot.frontRightMotor.getPower());
    }
    private void servoControl(){
        //0 = 0 degrees     1 = 180 degrees  value = degree/180
        if (gamepad1.b){
            robot.openGrabber();
        }
        else if(gamepad1.a){
            robot.closeGrabber();
        }
        //0 = in, 1 = out, 0.5 is stop
        double crservoscale=0.1;
        if(gamepad1.right_trigger>0){
            CRServoPower=0.5+gamepad1.right_trigger*0.5*crservoscale;
        }
        else if(gamepad1.left_trigger>0){
            CRServoPower=0.5-gamepad1.left_trigger*0.5*crservoscale;
        }
        else{
            CRServoPower=0.5;
        }
        robot.servo1.setPosition(CRServoPower);
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
    public int armmax = 350;
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
                robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(gamepad1.dpad_up){
            robot.liftMotor.setPower(-0.8);

        }
        else if(gamepad1.dpad_down){
            robot.liftMotor.setPower(0.2);

        }
        else{
            robot.liftMotor.setPower(-0.3);
        }
        telemetry.addData("Lift Motor Power:", robot.liftMotor.getPower());

    }
    @Override
    public void loop() {
        processDriveMotors();
        servoControl();
        liftMotorControl();
        //liftMotorStateMachine();
        telemetry.addData("Current BackLeftMotor Counts:", (robot.backLeftMotor.getCurrentPosition()));
        telemetry.addData("Current FrontLeftMotor Counts:", (robot.frontLeftMotor.getCurrentPosition()));
        telemetry.addData("Current BackRightMotor Counts:", (robot.backRightMotor.getCurrentPosition()));
        telemetry.addData("Current FrontRightMotor Counts:", (robot.frontRightMotor.getCurrentPosition()));
    }

    public void stop() {
        robot.reset();
        }

}