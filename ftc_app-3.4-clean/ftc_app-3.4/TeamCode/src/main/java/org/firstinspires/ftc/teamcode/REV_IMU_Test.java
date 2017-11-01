package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


/**
 * Created by Abhishek Vangipuram on 10/24/2017.
 */
@Autonomous (name="REV_IMU_Test")
public class REV_IMU_Test extends LinearOpMode {
    RobotHardware robot=new RobotHardware();
    Orientation angles;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        waitForStart();
        while (opModeIsActive()) {
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.update();
            //^^from the sample program
           robot.driveStraight(0.5, 1);
            robot.turnDegrees(90);
            robot.resetMotors();
            robot.resetEncoderValues();
        }
    }
    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    }

