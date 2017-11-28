package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Abhishek Vangipuram on 11/22/2017.
 */
@Autonomous(name = "NEW_BlueShortSide",group = "New Auton")
public class BlueShortSideAutonNew extends LinearOpMode {
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
            robot.reset();
            //clamping on glyph
            robot.closeGrabber();
            sleep(1000);
            //picking up glyph
            robot.liftMotor.setPower(-0.5);
            sleep(700);
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
            robot.servo2.setPosition(0.8);
            sleep(700);
            robot.servo2.setPosition(0.5);
            telemetry.addData("Status:","Detecting Jewel Color");
            telemetry.update();
            Color.RGBToHSV((int) (robot.colorsensor.red() * SCALE_FACTOR),
                    (int) (robot.colorsensor.green() * SCALE_FACTOR),
                    (int) (robot.colorsensor.blue() * SCALE_FACTOR),
                    hsvValues);
            //adding color sensor telemetry
            //telemetry.addData("Alpha", robot.colorsensor.alpha());//opacity, from 0 being fully transparent, to 1 being fully opaque
            telemetry.addData("Red  ", robot.colorsensor.red());
            //telemetry.addData("Green", robot.colorsensor.green());
            telemetry.addData("Blue ", robot.colorsensor.blue());
            //telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
            if(robot.colorsensor.blue()>robot.colorsensor.red()){
                telemetry.addData("Color:","Blue");
                telemetry.update();
                //REPLACE WITH METHOD THAT DRIVES FORWARD WITH TIME
                robot.setDrivePower(0.1,0.1);
                sleep(1000);
                robot.resetMotors();
                robot.servo2.setPosition(0);
                sleep(600);
                robot.servo2.setPosition(0.5);
            }
            else{
                telemetry.addData("Color:","Red");
                telemetry.update();
                //REPLACE WITH METHOD THAT DRIVES FORWARD WITH TIME
                robot.setDrivePower(-0.1,-0.1);
                sleep(1000);
                robot.resetMotors();
                robot.servo2.setPosition(0);
                sleep(600);
                robot.servo2.setPosition(0.5);
                robot.setDrivePower(0.1,0.1);
                //change this time so that it is in the same position as if it were th eother jewel
                sleep(1000);
            }
            telemetry.clearAll();
            telemetry.addData("Status:","Moving to Crypto-Box");
            telemetry.update();
            switch (curentVuMark){
                case UNKNOWN:
                case LEFT:
                    break;
                case CENTER:
                    break;
                case RIGHT:
                    break;
            }
            //robot.turnDegrees(90);
            telemetry.addData("Status:","Placing Glyph in Crypto-Box");
            telemetry.update();
            //PUT METHOD THAT GOES FORWARD WITH TIME
            //robot.liftMotor.setPower(0);
            sleep(500);
            //robot.openGrabber();
            telemetry.addData("Status:","Parking in Safe Zone");
            telemetry.update();
            //PUT METHOD THAT GOES BACKWARD WITH TIME


        }
        catch(InterruptedException e){

        }
        finally {
            robot.resetMotorsAndEncoders();
            telemetry.clearAll();
            telemetry.addData("Status:","Auton Finished!");
            telemetry.update();
        }


    }

}
