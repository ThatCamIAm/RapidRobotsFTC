package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Abhishek Vangipuram on 8/28/2017.
 */
public class RobotHardware {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor andyMarkMotor;
    public Servo servo1;
    public Servo servo2;
    final double FORWARD_SPEED = 0.3;
    final double TURN_SPEED    = 0.2;
    public final int ANDYMARK_REVOLUTION = 1120;
    public final int TETRIX_REVOLUTION = 1440;
    public final double WHEEL_DIAMETER = 4.0;
    public final double COUNTS_PER_INCH = ANDYMARK_REVOLUTION/(WHEEL_DIAMETER*Math.PI);
    public DcMotor getFrontLeft()
    {
        return frontLeftMotor;
    }
    public DcMotor getFrontRight()
    {
        return frontRightMotor;
    }
    public DcMotor getBackLeft()
    {
        return backLeftMotor;
    }
    public DcMotor getBackRight()
    {
        return backRightMotor;
    }
    public DcMotor getAndyMarkMotor(){return andyMarkMotor;}
    public HardwareMap hwMap = null;
    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        frontLeftMotor = hwMap.dcMotor.get("frontLeft");
        frontRightMotor = hwMap.dcMotor.get("frontRight");
        backLeftMotor = hwMap.dcMotor.get("backLeft");
        backRightMotor = hwMap.dcMotor.get("backRight");
        //servo for the balancing stone
        servo1=hwMap.servo.get("servo1");
        //servo for displacing the jewels
        servo2=hwMap.servo.get("servo2");
        // Set all motors to zero power
        servo1.setPosition(0);
        servo2.setPosition(0);
        resetMotors();
        resetEncoderValues();


    }
    public void resetMotors(){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void reset(){
        resetMotors();
        resetEncoderValues();
    }
    public void setDrivePower(double left, double right) {
        frontRightMotor.setPower(right);
        backRightMotor.setPower(right);
        frontLeftMotor.setPower(left);
        backLeftMotor.setPower(left);
    }
    protected void driveForward(){setDrivePower( FORWARD_SPEED,FORWARD_SPEED);}
    protected void driveBackward(){setDrivePower( -FORWARD_SPEED,-FORWARD_SPEED);}
    protected void turnRight() {setDrivePower(TURN_SPEED,-TURN_SPEED);}
    protected void turnLeft() {setDrivePower(-TURN_SPEED,TURN_SPEED);}
    protected void driveForwardInches(double inches, double power){
        //make inches negative to go backwards
        resetEncoderValues();
        int counts = (int)Math.round(COUNTS_PER_INCH*inches);
        frontRightMotor.setTargetPosition(counts);
        backRightMotor.setTargetPosition(counts);
        frontLeftMotor.setTargetPosition(counts);
        backLeftMotor.setTargetPosition(counts);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(power,power);
    }
    protected void resetEncoderValues(){
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
