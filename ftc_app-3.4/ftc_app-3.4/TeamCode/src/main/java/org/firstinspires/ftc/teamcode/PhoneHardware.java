package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PhoneHardware {
    HardwareMap hwMap = null;
public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        reset();
    }

    //resets all motor activity
    public void reset() {
    }

    }
