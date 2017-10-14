package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Created by Abhishek Vangipuram on 10/10/2017.
 */
@Autonomous (name = "REVColorDistanceTest")
public class REVColorDistanceTest  extends LinearOpMode{
    ColorSensor colorsensor;
    DistanceSensor distancesensor;
    @Override
    public void runOpMode() throws InterruptedException {
        colorsensor = hardwareMap.colorSensor.get("color_dist_sensor");
        distancesensor = hardwareMap.get(DistanceSensor.class, "color_dist_sensor");
        float hsvValues[] = {0F,0F,0F};
        // hsvValues is an array that will hold the hue, saturation, and value information.

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        waitForStart();
        while (opModeIsActive()){
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (colorsensor.red() * SCALE_FACTOR),
                    (int) (colorsensor.green() * SCALE_FACTOR),
                    (int) (colorsensor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", distancesensor.getDistance(DistanceUnit.INCH)));
            telemetry.addData("Alpha", colorsensor.alpha());//opacity, from 0 being fully transparent, to 1 being fully opaque
            telemetry.addData("Red  ", colorsensor.red());
            telemetry.addData("Green", colorsensor.green());
            telemetry.addData("Blue ", colorsensor.blue());
            telemetry.addData("Hue", hsvValues[0]);
        }
    }
}
