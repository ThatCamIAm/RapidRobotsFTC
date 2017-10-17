package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="BlueShortSide")
public class BlueShortSideAuton extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    //SET DEBUGWAIT TO ZERO FOR NORMAL RUN
    static int debugWait = 1000;
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
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
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
            telemetry.addData("Status", "Detecting Jewel Color");
            telemetry.update();
            sleep(debugWait);
            telemetry.addData("Status", "Displacing Jewel");
            telemetry.update();
            robot.servo2.setPosition(.6);
            sleep(100);
            robot.servo2.setPosition(.4);
            sleep(debugWait);
            telemetry.addData("Status", "Move to Crypto-Box Position-%s", currentVuMark);
            robot.driveForwardInches(10,.5);
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