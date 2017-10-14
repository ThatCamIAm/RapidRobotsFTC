package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;


    public DcMotor getFrontLeft() {return frontLeftMotor;}
    public DcMotor getFrontRight() {return frontRightMotor;}
    public DcMotor getBackleft() {return backLeftMotor;}
    public DcMotor getBackRight() {return backRightMotor;}

    HardwareMap hwMap = null;

    //initialize hardware interfaces
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        frontLeftMotor = hwMap.dcMotor.get("frontLeft");
        frontRightMotor = hwMap.dcMotor.get("frontRight");
        backLeftMotor = hwMap.dcMotor.get("backLeft");
        backRightMotor = hwMap.dcMotor.get("backRight");
        reset();
    }
    //resets all motor activity
    public void reset(){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void setDrivePower(double left,double right) {
        frontRightMotor.setPower(right);
        backRightMotor.setPower(right);
        frontLeftMotor.setPower(left);
        backLeftMotor.setPower(left);

    }

}
