package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/**
 * Created by Abhishek Vangipuram on 10/24/2017.
 */
@Autonomous (name="REV_IMU_Test")
public class REV_IMU_Test extends LinearOpMode {
    RobotHardware robot=new RobotHardware();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()){
            /*telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.firstAngle);
                        }
                    });*/
            //^^^is exactly from the sample program
            telemetry.addData("heading", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });

        }
        String formatAngle(AngleUnit angleUnit, double angle) {
            return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
        }

        String formatDegrees(double degrees){
            return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        }
        //^^from the sample program

        /*robot.driveStraight(0.5,2);
        robot.turnDegrees(90);

        robot.resetMotors();
        robot.resetEncoderValues();*/
        }
    }

