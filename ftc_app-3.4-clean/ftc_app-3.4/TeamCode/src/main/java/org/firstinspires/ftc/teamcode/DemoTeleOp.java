package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


    @TeleOp(name = "DemoTeleOp")
    public class DemoTeleOp extends OpMode {

        private RobotHardware robot = new RobotHardware();
        Servo servo1;

        @Override
        public void init() {
            robot.init(hardwareMap);
            servo1=hardwareMap.servo.get("servo1");
        }

        private void processDriveMotors() {
            float throttle = gamepad1.left_stick_y;
            float direction = gamepad1.left_stick_x;

            float rightPower = throttle - direction;
            float leftPower = direction + throttle;
            //restricting the values so they stay within -1 and 1
            leftPower = Range.clip(leftPower, -1, 1);
            rightPower = Range.clip(rightPower, -1, 1);

            robot.setDrivePower(-rightPower, leftPower);
        }


        @Override
        public void loop() {
            processDriveMotors();
            if (gamepad1.a) {
                servo1.setPosition(0.5);//0 = 0 degrees     1 = 180 degrees  value = degree/180
            }
            if (gamepad1.b) {
                servo1.setPosition(0);
            }
        }

        public void stop() {
            robot.reset();
        }
    }

