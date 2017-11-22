package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by RoboticsAcc on 11/21/2017.
 */
@TeleOp(name="ODS_Test", group = "Tests")
public class ODS_Test extends OpMode{
    OpticalDistanceSensor ods;
    @Override
    public void init() {
        ods=hardwareMap.opticalDistanceSensor.get("ods");
        ods.enableLed(true);
    }

    @Override
    public void loop() {
        telemetry.addData("ODS Light Value:", ods.getLightDetected());
    }

    @Override
    public void stop() {
        ods.close();
    }
}
