package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.util.Range;

@TeleOp(name ="DemoTeleOp")
public class DemoTeleOp extends OpMode {

    private RobotHardware robot = new RobotHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    private void processDriveMotors(){
        float throttle = -gamepad1.left_stick_y;
        float direction = -gamepad1.right_stick_x;

        float rightPower = throttle - direction;
        float leftPower = direction + throttle;
        //restricting the values so they stay within -1 and 1
        leftPower = Range.clip(leftPower, -1,1);
        rightPower = Range.clip(rightPower, -1,1);

        robot.setDrivePower(-rightPower,leftPower);

    }



    @Override
    public void loop() {
        processDriveMotors();
    }
    public void stop(){robot.reset();}
}
