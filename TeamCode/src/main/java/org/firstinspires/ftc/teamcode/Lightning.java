package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lightning {
    DcMotor svet;

    public Lightning(HardwareMap hardwareMap) {
        svet = hardwareMap.dcMotor.get("svet");
    }

    void setPower(double x) {
        svet.setPower(x);
    }
    void smooth(){
        double t = System.currentTimeMillis() / 1000.0;
        setPower((Math.sin(t)+1)/2);
    }
}
