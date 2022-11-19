package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lightning {
    DcMotor svet1;
    DcMotor svet2;

    public Lightning(HardwareMap hardwareMap) {
        svet1 = hardwareMap.dcMotor.get("svet1");
        svet2 = hardwareMap.dcMotor.get("svet2");
        svet1.setDirection(DcMotorSimple.Direction.FORWARD);
        svet2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPower(double x)
    {
        svet1.setPower(x);
        svet2.setPower(x);
    }

    public void smooth() {
        double t = System.currentTimeMillis() / 1000.0;
        setPower((Math.sin(t) + 1) / 2);
    }
}
