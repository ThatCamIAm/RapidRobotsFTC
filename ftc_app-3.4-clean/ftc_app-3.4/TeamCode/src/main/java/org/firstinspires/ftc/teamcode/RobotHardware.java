package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.String;


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
    public Servo servo3;
    BNO055IMU imu;
    Orientation angles;
    public final int ANDYMARK_REVOLUTION = 1120;
    public final int TETRIX_REVOLUTION = 1440;
    public final double WHEEL_DIAMETER = 4.0;
    public final double COUNTS_PER_INCH = ANDYMARK_REVOLUTION/(WHEEL_DIAMETER*Math.PI);
    //CPI is about 90 with current math
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
    //LinearOpMode LinearOpMode = new LinearOpMode;
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
        //adding servo for clamp
        servo3=hwMap.servo.get("servo3");
        //adding rev imu (gyro,accelerometer,etc.)
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles=imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
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
        servo1.setPosition(0);
        servo2.setPosition(0);
        servo3.setPosition(0);
    }
    public void setDrivePower(double left, double right) {
        frontLeftMotor.setPower(left);
        backLeftMotor.setPower(left);
        frontRightMotor.setPower(right);
        backRightMotor.setPower(right);
    }
    /*
    final double FORWARD_SPEED = 0.3;
    final double TURN_SPEED    = 0.2;
    protected void driveForward(){setDrivePower( FORWARD_SPEED,FORWARD_SPEED);}
    protected void driveBackward(){setDrivePower( -FORWARD_SPEED,-FORWARD_SPEED);}
    protected void turnRight() {setDrivePower(TURN_SPEED,-TURN_SPEED);}
    protected void turnLeft() {setDrivePower(-TURN_SPEED,TURN_SPEED);}
    */
    protected void driveForwardInches(double inches, double power){
        //make inches/counts negative to go backwards
        resetEncoderValues();
        int counts = (int)Math.round(COUNTS_PER_INCH*inches);
        frontRightMotor.setTargetPosition(-counts);
        backRightMotor.setTargetPosition(-counts);
        frontLeftMotor.setTargetPosition(counts);
        backLeftMotor.setTargetPosition(counts);
        setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(power,power);
        while (frontLeftMotor.isBusy()||frontRightMotor.isBusy()||backLeftMotor.isBusy()||backRightMotor.isBusy()){

        }
        resetMotors();
        resetEncoderValues();
    }
    protected void resetEncoderValues(){
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //LinearOpMode.waitOneFullHardwareCycle();
    }
    protected void setEncoderMode(DcMotor.RunMode mode){
        frontRightMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        backRightMotor.setMode(mode);
        frontLeftMotor.setMode(mode);
    }
    public void driveStraight(double power){
        double tolerance=5;
        //ElapsedTime runtime=new ElapsedTime();
        //if(imu.isGyroCalibrated()){
            //runtime.reset();
        int i=0;
            while(i<100000){
                if(Math.abs(angles.firstAngle)<=tolerance){
                    setDrivePower(power,power);
                }

                else if(Math.abs(angles.firstAngle)>tolerance&&angles.firstAngle<0){
                    while(Math.abs(angles.firstAngle)>tolerance){
                        setDrivePower(0.1,0.3);
                        //LinearOpMode.waitOneFullHardwareCycle();//<--comment out or delete this if bad
                                                                    //is deprecated, I don't know how it will work
                    }
                }
                else if(Math.abs(angles.firstAngle)>tolerance&&angles.firstAngle>0) {
                    while(Math.abs(angles.firstAngle)>tolerance) {
                        setDrivePower(0.3, 0.1);
                        //LinearOpMode.waitOneFullHardwareCycle();//<--comment out or delete this if bad
                                                                    //is deprecated, I don't know how it will work
                    }
                }
                //LinearOpMode.waitOneFullHardwareCycle();//<--comment out or delete this if bad
                                                            //is deprecated, I don't know how it will work
                i++;
            }
            resetMotors();
            resetEncoderValues();
        //}
       /* else{
            resetMotors();
            resetEncoderValues();
        }*/

        /*if(imu.isGyroCalibrated()){
            setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER); //keep this mode if encoders work

            while(Math.abs(angles.firstAngle)>tolerance){

            }
        }*/
            /*//if(Math.abs(angles.firstAngle)<=tolerance){
                while(Math.abs(angles.firstAngle)<=tolerance){
                    setDrivePower(0.1,0.3);
                //}
            }
            //if (Math.abs(angles.firstAngle)>=tolerance){
                while(Math.abs(angles.firstAngle)>=tolerance){
                    setDrivePower(0.3,0.1);
                //}
            }
            //else{setDrivePower(power,power);}
        }
        //else{
            setDrivePower(power,power);
        //}*/

    }
    public void turnDegrees(float degrees){
        double tolerance;
        if(imu.isGyroCalibrated()){
            setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER); //keep this mode if encoders work
            //if firstAngle is negative, the gyro is turning right
            //if firstAngle is positive, the gyro is turning left
            if(angles.firstAngle<degrees){
                while(angles.firstAngle<degrees){
                    setDrivePower(-0.2,0.2);
                    //LinearOpMode.waitOneFullHardwareCycle(); //<--comment out or delete this if bad
                                                                //is deprecated, I don't know how it will work
                }
                resetMotors();
                resetEncoderValues();
            }
            else if(angles.firstAngle>degrees){
                while(angles.firstAngle>degrees){
                    setDrivePower(0.2,-0.2);
                    //LinearOpMode.waitOneFullHardwareCycle();//<--comment out or delete this if bad
                                                                //is deprecated, I don't know how it will work
                }
                resetMotors();
                resetEncoderValues();
            }
            /*//if(degrees>0){
                while(angles.firstAngle<degrees){
                    setDrivePower(-0.2,0.2);
                }
            //}
            //if(degrees<0){
                while(angles.firstAngle>degrees){
                    setDrivePower(0.2,-0.2);
                }
                resetMotors();
            //}
            //set angles.firstAngle to 0 by calibrating the sensor to have a new zero
            //as otherwise, you will have to turn based off the first 0, or where we start our robot
            //place this recalibration code HERE
        }
        else{
            //turnDegreesEncoder --> make a method that turns based on encoders
            //turn at a certain power for 1 second, for left and right, find how many degrees it turns
            //and using math, find the time to turn per degree with encoders
            */
        }
        else{
            double dist_between_wheels;
            double rotation_circumference;
            double counts_per_degree;
            //encoder CPD stuff
        }


    }
}
