package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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

import java.util.Locale;

import static java.lang.Thread.currentThread;
import static java.lang.Thread.sleep;


/**
 * Created by Abhishek Vangipuram on 8/28/2017.
 */
public class RobotHardware {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor liftMotor;
    public Servo servo1;
    public Servo servo2;
    public Servo servo3;
    public Servo servo4;
    ColorSensor colorsensor;
    DistanceSensor distancesensor;
    ColorSensor colorsensor2;
    DistanceSensor distancesensor2;
    BNO055IMU imu;
    Orientation angles;
    ElapsedTime runtime= new ElapsedTime();
    Telemetry localtelemetry;
    public final int ANDYMARK_REVOLUTION = 1120;
    public final int TETRIX_REVOLUTION = 1440;
    public final double WHEEL_DIAMETER = 4.0;
    //CPI is about 90 with current math
    public final double COUNTS_PER_INCH = ANDYMARK_REVOLUTION / (WHEEL_DIAMETER * Math.PI);
    //INCHES_PER_SECOND IS FOR 0.2 POWER
    public DcMotor getFrontLeft() {
        return frontLeftMotor;
    }

    public DcMotor getFrontRight() {
        return frontRightMotor;
    }

    public DcMotor getBackLeft() {
        return backLeftMotor;
    }

    public DcMotor getBackRight() {
        return backRightMotor;
    }

    public DcMotor getLiftMotor() {
        return liftMotor;
    }

    //LinearOpMode LinearOpMode = new LinearOpMode;
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        localtelemetry = telemetry;
        // Save reference to Hardware map
        //drive motors
        frontLeftMotor = hwMap.dcMotor.get("frontLeft");
        frontRightMotor = hwMap.dcMotor.get("frontRight");
        backLeftMotor = hwMap.dcMotor.get("backLeft");
        backRightMotor = hwMap.dcMotor.get("backRight");
        //configuring liftmotor hardware map
        liftMotor = hwMap.dcMotor.get("liftMotor");
        //servo for displacing the jewels
        //ADD BACK WHEN SERVO IS FIXED
        //1 is out, 0 is all the way inside the robot, never set position to 0
        servo1 = hwMap.servo.get("servo1");
        //servo for displacing the jewels
        //1 is all the way in, 0 is out
        servo2 = hwMap.servo.get("servo2");
        //adding servo for clamp
        servo3 = hwMap.servo.get("servo3");
        servo4=hwMap.servo.get("servo4");
        //adding rev imu (gyro,accelerometer,etc.)
        colorsensor = hwMap.colorSensor.get("color_dist_sensor");
        distancesensor = hwMap.get(DistanceSensor.class, "color_dist_sensor");
        colorsensor2 = hwMap.colorSensor.get("color_dist_sensor2");
        distancesensor2 = hwMap.get(DistanceSensor.class, "color_dist_sensor2");
        //adding color and distance sensor
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new BasicAccelerationIntegrator();
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Set all motors to zero power
        runtime.reset();
        resetMotorsAndEncoders();


    }

    //----------------------------------------------------------------------------------------------
    // Angle Measurement Formatting
    //----------------------------------------------------------------------------------------------
    public double getCurrentAngle(){
        double anglesNorm;
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        anglesNorm= NormalizeDegrees(angles.angleUnit , angles.firstAngle);
        return anglesNorm;
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public double NormalizeDegrees(AngleUnit angleUnit, double angle){
        double degrees;
        degrees = AngleUnit.DEGREES.fromUnit(angleUnit, angle);
        degrees = AngleUnit.DEGREES.normalize(degrees);
        return degrees;
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }



    public void resetMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void reset() {
        resetMotorsAndEncoders();
        //ADD BACK WHEN SERVO IS FIXED
        servo1.setPosition(0.3);
        servo2.setPosition(0.7);
        servo3.setPosition(.4);
        servo4.setPosition(.6);
    }
    public void resetMotorsAndEncoders() {
        resetMotors();
        resetEncoderValues();
    }

    public void setDrivePower(double left, double right) {
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
    protected void driveBackwardInches(double inches, double power, double timeout) {
        resetMotorsAndEncoders();
        int tolerance=20;
        int counts = (int) Math.round(COUNTS_PER_INCH * inches);
//must set direction first
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//then set position
        backRightMotor.setTargetPosition(counts);
        frontLeftMotor.setTargetPosition(counts);
//then set the mode
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//then set the desired power
        runtime.reset();
        //frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(power);
        //backLeftMotor.setPower(power);



        while(Math.abs(frontLeftMotor.getTargetPosition()-frontLeftMotor.getCurrentPosition())>tolerance||Math.abs(backRightMotor.getTargetPosition()-backRightMotor.getCurrentPosition())>tolerance) {
            if(runtime.seconds()>timeout){
                break;
            }
            backLeftMotor.setPower(frontLeftMotor.getPower());
            frontRightMotor.setPower(backRightMotor.getPower());
            localtelemetry.addData("Current BackLeftMotor Counts:", (backLeftMotor.getCurrentPosition()));
            localtelemetry.addData("Current FrontLeftMotor Counts:", (frontLeftMotor.getCurrentPosition()));
            localtelemetry.addData("Current BackRightMotor Counts:", (backRightMotor.getCurrentPosition()));
            localtelemetry.addData("Current FrontRightMotor Counts:", (frontRightMotor.getCurrentPosition()));
            localtelemetry.addData("BackLeftMotor Power:", backLeftMotor.getPower());
            localtelemetry.addData("FrontLeftMotor Power:", frontLeftMotor.getPower());
            localtelemetry.addData("BackRightMotor Power:", backRightMotor.getPower());
            localtelemetry.addData("FrontRIghtMotor Power:", frontRightMotor.getPower());
            localtelemetry.addData("FrontLeft Target Pos:", frontLeftMotor.getTargetPosition());
            localtelemetry.addData("BackRight Target Pos:", backRightMotor.getTargetPosition());
            localtelemetry.update();
        }
        resetMotorsAndEncoders();

    }
    protected void openGrabber(){
        servo3.setPosition(.25);
        servo4.setPosition(.75);
    }
    protected void closeGrabber(){
        servo3.setPosition(0);
        servo4.setPosition(1);
    }

    protected void driveForwardInches(double inches, double power,double timeout) {
        resetMotorsAndEncoders();

        int tolerance=20;
        int counts = (int) Math.round(COUNTS_PER_INCH * inches);
        //must set direction first
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //then set position
        backRightMotor.setTargetPosition(counts);
        frontLeftMotor.setTargetPosition(counts);
        //then set the mode
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(power);

        while(Math.abs(frontLeftMotor.getTargetPosition()-frontLeftMotor.getCurrentPosition())>tolerance||Math.abs(backRightMotor.getTargetPosition()-backRightMotor.getCurrentPosition())>tolerance) {
                if(runtime.seconds()>timeout){
                    break;
                }
                frontRightMotor.setPower(backRightMotor.getPower());
                backLeftMotor.setPower(frontLeftMotor.getPower());
                localtelemetry.addData("Current BackLeftMotor Counts:", (backLeftMotor.getCurrentPosition()));
                localtelemetry.addData("Current FrontLeftMotor Counts:", (frontLeftMotor.getCurrentPosition()));
                localtelemetry.addData("Current BackRightMotor Counts:", (backRightMotor.getCurrentPosition()));
                localtelemetry.addData("Current FrontRightMotor Counts:", (frontRightMotor.getCurrentPosition()));
                localtelemetry.addData("BackLeftMotor Power:", backLeftMotor.getPower());
                localtelemetry.addData("FrontLeftMotor Power:", frontLeftMotor.getPower());
                localtelemetry.addData("BackRightMotor Power:", backRightMotor.getPower());
                localtelemetry.addData("FrontRIghtMotor Power:", frontRightMotor.getPower());
                localtelemetry.addData("FrontLeft Target Pos:", frontLeftMotor.getTargetPosition());
                localtelemetry.addData("BackRight Target Pos:", backRightMotor.getTargetPosition());
                localtelemetry.update();
            }
            resetMotorsAndEncoders();
        }



    protected void resetEncoderValues() {
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //LinearOpMode.waitOneFullHardwareCycle();
    }

    protected void setEncoderMode(DcMotor.RunMode mode) {
        frontRightMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        backRightMotor.setMode(mode);
        frontLeftMotor.setMode(mode);
    }

    public void driveStraight(double power,double time) {
        resetMotorsAndEncoders();
        double startingAngle = getCurrentAngle();
        double tolerance=5;
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();
        while (runtime.seconds()<time){
            double localAngle =getCurrentAngle();
            localtelemetry.addData("Runtime:",runtime.seconds());
            localtelemetry.addData("Heading:",localAngle);
            localtelemetry.addData("Left Motor Power:", frontLeftMotor.getPower());
            localtelemetry.addData("Right Motor Power:",frontRightMotor.getPower());
            localtelemetry.update();
            if(Math.abs(localAngle-startingAngle)<=tolerance){
                setDrivePower(power,power);
            }
            else{
                if ((localAngle-startingAngle)>0){
                    setDrivePower(power,power*0.5);
                }
                else{
                    setDrivePower(power*0.5,power);
                }
            }
        }
        resetMotorsAndEncoders();

//-------------------------------------------------------------------------------------------------------------------------------------
        /*double tolerance = 5;
        //ElapsedTime runtime=new ElapsedTime();
        //if(imu.isGyroCalibrated()){
        //runtime.reset();
        int i = 0;
        while (i < 100000) {
            if (Math.abs(angles.firstAngle) <= tolerance) {
                setDrivePower(power, power);
            } else if (Math.abs(angles.firstAngle) > tolerance && angles.firstAngle < 0) {
                while (Math.abs(angles.firstAngle) > tolerance) {
                    setDrivePower(0.1, 0.3);
                    //LinearOpMode.waitOneFullHardwareCycle();//<--comment out or delete this if bad
                    //is deprecated, I don't know how it will work
                }
            } else if (Math.abs(angles.firstAngle) > tolerance && angles.firstAngle > 0) {
                while (Math.abs(angles.firstAngle) > tolerance) {
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
        resetEncoderValues();*/
        //}
       /* else{
            resetMotors();
            resetEncoderValues();
        }*/
//-------------------------------------------------------------------------------------------------------------------------------------
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
//-------------------------------------------------------------------------------------------------------------------------------------
    }
    public void turnToDegrees(float degrees){
        resetMotorsAndEncoders();
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        double tolerance = 5;
        //wrap final Angle to +/- 180
        if (degrees > 180)
            degrees -= 360;
        if (degrees <= -180)
            degrees += 360;

        setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(degrees>0){
            setDrivePower(-0.2,0.2);
        }
        else {
            setDrivePower(0.2,-0.2);
        }
        while (Math.abs(degrees-getCurrentAngle())>tolerance){
            localtelemetry.addData("Heading:",getCurrentAngle());
            localtelemetry.addData("Target Angle",degrees);
            localtelemetry.update();
        }
        resetMotorsAndEncoders();

    }

    public void turnDegreesPower(float degrees, double power){
        resetMotorsAndEncoders();
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        double startingAngle = getCurrentAngle();
        double finalAngle = startingAngle + degrees;
        double tolerance = 3;
        //wrap final Angle to +/- 180
        if (finalAngle > 180)
            finalAngle -= 360;
        if (finalAngle <= -180)
            finalAngle += 360;

        setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(finalAngle>startingAngle){
            setDrivePower(power,-power);
        }
        else {
            setDrivePower(-power,power);
        }
        while (Math.abs(finalAngle-getCurrentAngle())>tolerance){
            localtelemetry.addData("Heading:",getCurrentAngle());
            localtelemetry.addData("start angle:",startingAngle);
            localtelemetry.addData("final angle:",finalAngle);
            localtelemetry.update();
        }
        resetMotorsAndEncoders();
    }
    public void turnDegrees(float degrees) {
        resetMotorsAndEncoders();
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        double startingAngle = getCurrentAngle();
        double finalAngle = startingAngle + degrees;
        double tolerance = 3;
        //wrap final Angle to +/- 180
        if (finalAngle > 180)
            finalAngle -= 360;
        if (finalAngle <= -180)
            finalAngle += 360;

        setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(finalAngle>startingAngle){
            setDrivePower(0.1,-0.1);
        }
        else {
            setDrivePower(-0.1,0.1);
        }
        while (Math.abs(finalAngle-getCurrentAngle())>tolerance){
            localtelemetry.addData("Heading:",getCurrentAngle());
            localtelemetry.addData("start angle:",startingAngle);
            localtelemetry.addData("final angle:",finalAngle);
            localtelemetry.update();
            }
//-------------------------------------------------------------------------------------------------------------------------------------
            /*while(Math.abs(finalAngle - angles.firstAngle)<= tolerance){
                localtelemetry.addData("Heading:",angles.firstAngle);
                if (tolerance < degrees) {
                    setDrivePower(-0.2, 0.2);
                }
                else{
                    setDrivePower(0.2, -0.2);
                }

                resetMotors();
                resetEncoderValues();

            }*/
            resetMotorsAndEncoders();
        }
//-------------------------------------------------------------------------------------------------------------------------------------
        /*double tolerance;
        if (imu.isGyroCalibrated()) {
            setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER); //keep this mode if encoders work
            //if firstAngle is negative, the gyro is turning right
            //if firstAngle is positive, the gyro is turning left
            if (angles.firstAngle < degrees) {
                while (angles.firstAngle < degrees) {
                    setDrivePower(-0.2, 0.2);
                    //LinearOpMode.waitOneFullHardwareCycle(); //<--comment out or delete this if bad
                    //is deprecated, I don't know how it will work
                }
                resetMotors();
                resetEncoderValues();
            } else if (angles.firstAngle > degrees) {
                while (angles.firstAngle > degrees) {
                    setDrivePower(0.2, -0.2);
                    //LinearOpMode.waitOneFullHardwareCycle();//<--comment out or delete this if bad
                    //is deprecated, I don't know how it will work
                }
                resetMotors();
                resetEncoderValues();
            }
            */
    public void oldTurnDegrees(float degrees) {
        resetMotorsAndEncoders();
        double startingAngle = getCurrentAngle();
        double finalAngle = startingAngle + degrees;
        double tolerance = 3;
        //wrap final Angle to +/- 180
        if (finalAngle > 180)
            finalAngle -= 360;

        if (finalAngle <= -180)
            finalAngle += 360;


        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(finalAngle-getCurrentAngle())>tolerance){
            localtelemetry.addData("Heading:",getCurrentAngle());
            localtelemetry.addData("start angle:",startingAngle);
            localtelemetry.addData("final angle:",finalAngle);
            if(finalAngle>0){
                setDrivePower(-0.2,0.2);
            }
            else{
                setDrivePower(0.2,-0.2);
            }
            localtelemetry.update();
        }
        resetMotorsAndEncoders();
    }
    public void turnDegreesWithEncoders(float degrees){
        //TURNING WITH ENCODERS STUFF      DO NOT USE IF GYRO IS WORKING
        //FIND THE CORRECT DISTANCE BETWEEN WHEELS
        resetMotorsAndEncoders();
        double dist_between_wheels = 14.8;
        double rotation_circumference_in_encoder_counts = COUNTS_PER_INCH * dist_between_wheels * Math.PI;
        double counts_per_degree = rotation_circumference_in_encoder_counts / 360;

        if (degrees < 0) {
            //turns left (look at driveForwardInches to see why the sign of counts are weird)
            frontLeftMotor.setTargetPosition((int) (-degrees * counts_per_degree));
            frontRightMotor.setTargetPosition((int) (-degrees * counts_per_degree));
            backLeftMotor.setTargetPosition((int) (-degrees * counts_per_degree));
            backRightMotor.setTargetPosition((int) (-degrees * counts_per_degree));
        }
        if (degrees > 0) {
            //turns right (look at driveForwardInches to see why the sign of counts are weird)
            frontLeftMotor.setTargetPosition((int) (degrees * counts_per_degree));
            frontRightMotor.setTargetPosition((int) (degrees * counts_per_degree));
            backLeftMotor.setTargetPosition((int) (degrees * counts_per_degree));
            backRightMotor.setTargetPosition((int) (degrees * counts_per_degree));
        }
        setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(0.4, 0.4);
        while (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy()) {
            localtelemetry.addData("Current LeftMotor Counts", (backLeftMotor.getCurrentPosition()));
            localtelemetry.addData("Left Target Pos",backLeftMotor.getTargetPosition());
            localtelemetry.addData("Left Motor Power",backLeftMotor.getPower());
            localtelemetry.addData("Current RightMotor Counts", (backRightMotor.getCurrentPosition()));
            localtelemetry.addData("Right Target Pos",backRightMotor.getTargetPosition());
            localtelemetry.addData("Right Motor Power",backRightMotor.getPower());
            localtelemetry.update();
            }
        resetMotorsAndEncoders();

    }
    public void driveForwardInchesStraight(double inches, double power){
        resetMotorsAndEncoders();
        double startingAngle = getCurrentAngle();
        double tolerance=5;
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetMotorsAndEncoders();
        int counts = (int) Math.round(COUNTS_PER_INCH * inches);
        frontRightMotor.setTargetPosition(-counts);
        backRightMotor.setTargetPosition(-counts);
        frontLeftMotor.setTargetPosition(counts);
        backLeftMotor.setTargetPosition(counts);
        setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(power, power);
        while (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy()) {
            double localAngle = getCurrentAngle();
            if(Math.abs(localAngle-startingAngle)<=tolerance){
                setDrivePower(power,power);
            }
            else{
                if ((localAngle-startingAngle)>0){
                    setDrivePower(power,power*0.5);
                }
                else{
                    setDrivePower(power*0.5,power);
                }
            }
            localtelemetry.addData("Heading:",localAngle);
            localtelemetry.addData("Current LeftMotor Counts", (backLeftMotor.getCurrentPosition()));
            localtelemetry.addData("Left Target Pos",backLeftMotor.getTargetPosition());
            localtelemetry.addData("Left Motor Power",backLeftMotor.getPower());
            localtelemetry.addData("Current RightMotor Counts", (backRightMotor.getCurrentPosition()));
            localtelemetry.addData("Right Target Pos",backRightMotor.getTargetPosition());
            localtelemetry.addData("Right Motor Power",backRightMotor.getPower());
            localtelemetry.update();
        }
        resetMotorsAndEncoders();
    }

}


