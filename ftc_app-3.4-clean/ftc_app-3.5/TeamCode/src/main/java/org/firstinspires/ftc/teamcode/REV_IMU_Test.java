package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


/**
 * Created by Abhishek Vangipuram on 10/24/2017.
 */
@Autonomous (name="REV_IMU_Test", group = "Tests")
public class REV_IMU_Test extends LinearOpMode {
    RobotHardware robot=new RobotHardware();
    Orientation angles;
    Position position=new Position(DistanceUnit.INCH,0,0,0,20);
    Velocity velocity=new Velocity(DistanceUnit.INCH,0,0,0,20);
    Acceleration accel;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        waitForStart();
        robot.reset();
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        /*robot.turnDegrees(-90);
        sleep(1000);
        robot.turnDegrees(90);
        sleep(1000);
        */
        /*robot.oldTurnDegrees(90);
        sleep(500);
        robot.oldTurnDegrees(90);
        sleep(500);
        robot.turnDegrees(90);
        sleep(500);
        robot.turnDegrees(90);
        sleep(500);*/
        /*robot.turnDegrees(50);
        sleep(1000);
        robot.turnDegrees(111);
        sleep(1000);*/

        robot.imu.startAccelerationIntegration(position, velocity,20);
        while(opModeIsActive()){
            position = robot.imu.getPosition();
            velocity = robot.imu.getVelocity();
            accel=robot.imu.getAcceleration();
            telemetry.addData("",position.toString());
            telemetry.addData("Position X",position.x);
            telemetry.addData("Position Y",position.y);
            telemetry.addData("Position Z",position.z);
            telemetry.addData("Velocity X",velocity.xVeloc);
            telemetry.addData("Velocity Y",velocity.yVeloc);
            telemetry.addData("Velocity Z",velocity.zVeloc);
            telemetry.addData("Acceleration X",accel.xAccel);
            telemetry.addData("Acceleration Y",accel.yAccel);
            telemetry.addData("Acceleration Z",accel.zAccel);
            telemetry.update();
        }
        robot.imu.stopAccelerationIntegration();
        robot.resetMotorsAndEncoders();
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------
    private void driveForwardWithIMUPosition(double power,double inches, double tolerance){
        while(Math.abs(inches-position.x)>tolerance) {
            robot.setDrivePower(power, power);
        }
        robot.resetMotors();
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    //^^from the sample program
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    //^^from the sample program

}

