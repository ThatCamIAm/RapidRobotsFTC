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
    static int debugWait = 0;
    RelicRecoveryVuMark currentVuMark = RelicRecoveryVuMark.UNKNOWN;
    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    //instance of VumarkDetection
    RapidRobotsVuMarkDetection vuDetector = new RapidRobotsVuMarkDetection();


    private RobotHardware robot = new RobotHardware();
    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry);
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run"); 
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.reset();
        robot.driveForwardInches(3,.2);

        telemetry.addData("Status", "Detecting Crypto-Key");
        telemetry.update();
        //crypto key code, move init outside waitforstart later
        vuDetector.init(hardwareMap, telemetry);
        vuDetector.RunDetection();
        currentVuMark = vuDetector.getCryptoKey();
        //ADD CODE TO TURN OFF THE CAMERA

        telemetry.addData("Status", "Dropping Color Sensor arm");
        robot.servo2.setPosition(0);
        sleep(2000);
        telemetry.update();
        telemetry.addData("Status", "Detecting Jewel Color");
        Color.RGBToHSV((int) (robot.colorsensor.red() * SCALE_FACTOR),
                (int) (robot.colorsensor.green() * SCALE_FACTOR),
                (int) (robot.colorsensor.blue() * SCALE_FACTOR),
                hsvValues);
        //adding color sensor telemetry
        //telemetry.addData("Alpha", robot.colorsensor.alpha());//opacity, from 0 being fully transparent, to 1 being fully opaque
        //telemetry.addData("Red  ", robot.colorsensor.red());
        //telemetry.addData("Green", robot.colorsensor.green());
        //telemetry.addData("Blue ", robot.colorsensor.blue());
        //telemetry.addData("Hue", hsvValues[0]);
        if(robot.colorsensor.red()>robot.colorsensor.blue()){
            telemetry.addData("color:","Red");
        }
        else{
            telemetry.addData("color:","Blue");
        }
        telemetry.update();
        sleep(debugWait);
        telemetry.addData("Status", "Displacing Jewel");
        if (robot.colorsensor.red()<robot.colorsensor.blue()){
            robot.turnDegrees(-10);
            sleep(500);
            robot.turnDegrees(10);
            sleep(500);
            robot.servo2.setPosition(1);
        }
        else if(robot.colorsensor.red()>robot.colorsensor.blue()){
            robot.turnDegrees(10);
            sleep(500);
            robot.turnDegrees(-10);
            sleep(500);
            robot.servo2.setPosition(1);

        }
        else {
            robot.servo2.setPosition(1);

        }
        telemetry.update();
        sleep(debugWait);
        telemetry.addData("Status", "Move to Crypto-Box Position-%s", currentVuMark);
        robot.driveForwardInches(-3, .2);
        robot.turnDegrees(-90);
        telemetry.update();
        sleep(debugWait);
        telemetry.addData("Status", "Placing glyph in %s position", currentVuMark);
        switch (currentVuMark)
        {
            case UNKNOWN:
                //if unknown, assume center and continue
                    break;
            case CENTER:
                robot.driveForwardInches(36,.5);
                    break;
            case LEFT:
                robot.driveForwardInches(28.5,.5);
                break;
            case RIGHT:
                robot.driveForwardInches(44,.5);
                break;
            }
            robot.turnDegrees(90);
            
        telemetry.update();
        sleep(debugWait);
        telemetry.addData("Status","Parking in Safe Zone");
        telemetry.update();
        sleep(debugWait);

    }
}