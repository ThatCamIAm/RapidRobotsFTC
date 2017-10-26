package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="BlueShortSide")
public class BlueShortSideAuton extends LinearOpMode {

    //SET DEBUGWAIT TO ZERO FOR NORMAL RUN
    static int debugWait = 1000;
    RelicRecoveryVuMark currentVuMark = RelicRecoveryVuMark.UNKNOWN;
    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    //instance of VumarkDetection
    RapidRobotsVuMarkDetection vuDetector = new RapidRobotsVuMarkDetection();
    ColorSensor colorsensor;
    DistanceSensor distancesensor;
    private RobotHardware robot = new RobotHardware();
    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        colorsensor = hardwareMap.colorSensor.get("color_dist_sensor");
        distancesensor = hardwareMap.get(DistanceSensor.class, "color_dist_sensor");
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run"); 
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
            telemetry.addData("Status", "Detecting Crypto-Key");
            telemetry.update();
            //crypto key code, move init outside waitforstart later
            vuDetector.init(hardwareMap, telemetry);
            vuDetector.RunDetection();
            currentVuMark = vuDetector.getCryptoKey();
            //ADD CODE TO TURN OFF THE CAMERA
            sleep(debugWait);
        robot.servo2.setPosition(.3);
            telemetry.addData("Status", "Detecting Jewel Color");
        Color.RGBToHSV((int) (colorsensor.red() * SCALE_FACTOR),
                (int) (colorsensor.green() * SCALE_FACTOR),
                (int) (colorsensor.blue() * SCALE_FACTOR),
                hsvValues);
        //adding color sensor telemetry
        telemetry.addData("Alpha", colorsensor.alpha());//opacity, from 0 being fully transparent, to 1 being fully opaque
        telemetry.addData("Red  ", colorsensor.red());
        telemetry.addData("Green", colorsensor.green());
        telemetry.addData("Blue ", colorsensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();
            sleep(debugWait);
            telemetry.addData("Status", "Displacing Jewel");
        if (colorsensor.red()<colorsensor.blue()){
            robot.servo2.setPosition(.3);
            /*robot.setDrivePower(0.1,0.2);
            sleep(500);
            robot.resetMotors();
            robot.servo2.setPosition(0);
            robot.setDrivePower(0.1,0.2);
            sleep(500);
            robot.resetMotors();*/
        }
        else if(colorsensor.red()>colorsensor.blue()){
            robot.servo2.setPosition(.3);
            /*robot.setDrivePower(0.2,0.1);
            sleep(500);
            robot.resetMotors();
            robot.servo2.setPosition(0);
            robot.setDrivePower(0.2,0.1);
            sleep(500);
            robot.resetMotors();*/
        }
        else {
            //robot.setDrivePower(0,0);

        }
            telemetry.update();
            sleep(debugWait);
            telemetry.addData("Status", "Move to Crypto-Box Position-%s", currentVuMark);
            robot.driveForwardInches(10,.2);
            telemetry.update();
            sleep(debugWait);
            telemetry.addData("Status", "Placing glyph in %s position", currentVuMark);
            switch (currentVuMark)
            {
                case UNKNOWN:
                    //if unknown, assume center and continue
                case CENTER:

                    break;
                case LEFT:

                    break;
                case RIGHT:

                    break;
            }
            telemetry.update();
            sleep(debugWait);
            telemetry.addData("Status","Parking in Safe Zone");
            telemetry.update();
            sleep(debugWait);



    }
}