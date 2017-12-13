package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by RoboticsAcc on 11/16/2017.
 */
@Autonomous(name="EMERGENCYBlueLongSide", group="EMERGENCY Auton")
@Disabled
public class EMERGENCYBlueLongSideAuton extends LinearOpMode {
    //SET DEBUGWAIT TO ZERO FOR NORMAL RUN
    static int debugWait = 0;
    RelicRecoveryVuMark currentVuMark = RelicRecoveryVuMark.UNKNOWN;
    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    Orientation angles;

    //instance of VumarkDetection
    RapidRobotsVuMarkDetection vuDetector = new RapidRobotsVuMarkDetection();


    private RobotHardware robot = new RobotHardware();
    @Override
    public void runOpMode() throws InterruptedException {
        try {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
            robot.init(hardwareMap, telemetry);

            // hsvValues is an array that will hold the hue, saturation, and value information.
            float hsvValues[] = {0F, 0F, 0F};

            // values is a reference to the hsvValues array.

            // sometimes it helps to multiply the raw RGB values with a scale factor
            // to amplify/attentuate the measured values.
            final double SCALE_FACTOR = 255;

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Ready to run");
            telemetry.update();
            // Wait for the game to staart (driver presses PLAY)
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            waitForStart();
            robot.reset();
            robot.closeGrabber();
            sleep(1000);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.liftMotor.setPower(-0.5);
            sleep(700);
            robot.liftMotor.setPower(-0.3);
            telemetry.update();
            telemetry.addData("Status", "Dropping Color Sensor arm");
            robot.servo2.setPosition(.15);
            sleep(200);
            robot.servo2.setPosition(.3);
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
            telemetry.addData("Status", "Displacing Jewel");
            if (robot.colorsensor.red() < robot.colorsensor.blue()) {
                telemetry.addData("Color:", "Blue");
                telemetry.update();
                robot.turnDegrees(-10);
                sleep(500);
                robot.servo2.setPosition(.9);
                sleep(200);
                robot.turnDegrees(10);
            }
            else {
                telemetry.addData("Color:", "Red");
                telemetry.update();
                robot.turnDegrees(10);
                sleep(500);
                robot.servo2.setPosition(.9);
                sleep(200);
                robot.turnDegrees(-10);
            }
            telemetry.update();
            sleep(1000);
            telemetry.addData("Status", "Placing glyph in %s position", currentVuMark);
            robot.turnDegrees(80);
            robot.driveBackwardInches(20,0.3,5);
            robot.liftMotor.setPower(0);
            sleep(500);
            robot.openGrabber();
            telemetry.update();
            telemetry.addData("Status", "Parking in Safe Zone");
            telemetry.update();
            //robot.resetMotorsAndEncoders();
        } finally{
            robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.resetMotorsAndEncoders();
        }
    }

}
