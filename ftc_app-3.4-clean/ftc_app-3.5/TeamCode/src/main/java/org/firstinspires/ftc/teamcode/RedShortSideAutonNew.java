package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Abhishek Vangipuram on 11/22/2017.
 */
@Autonomous(name = "NEW_RedShortSide", group = "New Auton")
@Disabled
public class RedShortSideAutonNew extends LinearOpMode {
    private RobotHardware robot=new RobotHardware();
    private RelicRecoveryVuMark curentVuMark=RelicRecoveryVuMark.UNKNOWN;
    RapidRobotsVuMarkDetection vuDetetctor=new RapidRobotsVuMarkDetection();
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot.init(hardwareMap,telemetry);
            vuDetetctor.init(hardwareMap,telemetry);
            robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // hsvValues is an array that will hold the hue, saturation, and value information.
            float hsvValues[] = {0F, 0F, 0F};
            // sometimes it helps to multiply the raw RGB values with a scale factor
            // to amplify/attentuate the measured values.
            final double SCALE_FACTOR = 255;
            telemetry.addData("Status:","Ready to run");
            telemetry.update();
            waitForStart();
            robot.resetMotors();
            robot.servo2.setPosition(0.6);
            sleep(500);
            //clamping on glyph
            robot.closeGrabber();
            sleep(1000);
            //picking up glyph
            robot.liftMotor.setPower(-0.7);
            sleep(200);
            //keeping glyph up
            robot.liftMotor.setPower(-0.3);
            telemetry.addData("Status:","Detecting VuMark");
            telemetry.update();
            //Detecting vumark and storing value as LEFT, CENTER, or RIGHT
            vuDetetctor.RunDetection();
            curentVuMark=vuDetetctor.getCryptoKey();
            telemetry.addData("VuMark",curentVuMark);
            telemetry.update();
            telemetry.addData("Status:","Dropping Color Sensor Arm");
            telemetry.update();
            robot.servo1.setPosition(1);
            sleep(1000);
            robot.servo1.getController().pwmDisable();
            sleep(500);
            telemetry.addData("Status:","Detecting Jewel Color");
            telemetry.update();
            Color.RGBToHSV((int) (robot.colorsensor2.red() * SCALE_FACTOR),
                    (int) (robot.colorsensor2.green() * SCALE_FACTOR),
                    (int) (robot.colorsensor2.blue() * SCALE_FACTOR),
                    hsvValues);
            //adding color sensor telemetry
            //telemetry.addData("Alpha", robot.colorsensor2.alpha());//opacity, from 0 being fully transparent, to 1 being fully opaque
            telemetry.addData("Red  ", robot.colorsensor2.red());
            //telemetry.addData("Green", robot.colorsensor2.green());
            telemetry.addData("Blue ", robot.colorsensor2.blue());
            //telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
            if(robot.colorsensor2.blue()<robot.colorsensor2.red()){
                telemetry.addData("Color:","Red");
                telemetry.update();
                //REPLACE WITH METHOD THAT DRIVES FORWARD WITH TIME
                driveForwardInchesWithTime(6);
                /*robot.setDrivePower(0.1,0.1);
                sleep(1000);
                robot.resetMotors();
                */
                robot.servo1.getController().pwmEnable();
                sleep(500);
                robot.servo1.setPosition(0.4);
                sleep(500);
                driveForwardInchesWithTime(12);
                robot.liftMotor.setPower(-0.7);
                sleep(400);
                robot.liftMotor.setPower(-0.3);
                telemetry.clearAll();
                telemetry.addData("Status:","Moving to Crypto-Box");
                telemetry.update();
                switch (curentVuMark){
                    case UNKNOWN:
                    case LEFT:
                        driveForwardInchesWithTime(16);
                        robot.turnDegrees(-85);
                        break;
                    case CENTER:
                        driveForwardInchesWithTime(23);
                        robot.turnDegrees(-85);
                        break;
                    case RIGHT:
                        driveForwardInchesWithTime(32);
                        robot.turnDegrees(-85);
                        break;
                }

            }
            else{
                telemetry.addData("Color:","Blue");
                telemetry.update();
                robot.setDrivePower(0.2,0.2);
                sleep(400);
                robot.resetMotors();
                robot.servo1.getController().pwmEnable();
                sleep(500);
                robot.servo1.setPosition(0.4);
                sleep(600);
                //ROBOT IS BACKWARDS
                robot.setDrivePower(-0.2,-0.2);
                sleep(500);
                robot.resetMotors();
                sleep(2000);
                driveForwardInchesWithTime(14);
                robot.liftMotor.setPower(-0.7);
                sleep(400);
                robot.liftMotor.setPower(-0.3);
                telemetry.clearAll();
                telemetry.addData("Status:","Moving to Crypto-Box");
                telemetry.update();
                switch (curentVuMark){
                    case UNKNOWN:
                    case LEFT:
                        driveForwardInchesWithTime(18);
                        robot.turnDegrees(-85);
                        break;
                    case CENTER:
                        driveForwardInchesWithTime(25);
                        robot.turnDegrees(-85);
                        break;
                    case RIGHT:
                        driveForwardInchesWithTime(35);
                        robot.turnDegrees(-85);
                        break;
                }

            }
            //robot.turnDegrees(90);
            telemetry.addData("Status:","Placing Glyph in Crypto-Box");
            telemetry.update();
            //PUT METHOD THAT GOES FORWARD WITH TIME
            driveForwardInchesWithTime(2);
            robot.liftMotor.setPower(0.2);
            sleep(300);
            robot.liftMotor.setPower(0);
            sleep(500);
            robot.openGrabber();
            telemetry.addData("Status:","Parking in Safe Zone");
            telemetry.update();
            //PUT METHOD THAT GOES BACKWARD WITH TIME
            driveForwardInchesWithTime(-6);


        }
        catch(InterruptedException e){
            telemetry.addData("Status:","Auton Error Occured");
            telemetry.update();
        }
        finally {
            robot.resetMotorsAndEncoders();
            telemetry.addData("Status:","Auton Finished!");
            telemetry.update();
        }


    }
    public void driveForwardInchesWithTime(double inches){
        final double SECONDS_PER_INCH = 0.16;
        double timeDouble=1000*(Math.abs(inches)*SECONDS_PER_INCH);
        long timeLong= (long) timeDouble;
        if(inches>=0){
            robot.setDrivePower(-0.1,-0.1);
        }
        else{
            robot.setDrivePower(0.1,0.1);
        }
        sleep(timeLong);
        robot.resetMotors();
    }
}
