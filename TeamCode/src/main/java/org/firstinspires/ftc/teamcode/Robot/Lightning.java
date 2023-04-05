package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Lightning {

    DcMotor svet1;
    DcMotor svet2;

    public enum LightningMode {
        OFF, ON, SMOOTH;
    }

    public LightningMode lightMode = LightningMode.OFF;
    AiRRobot aiRRobot;

    public Lightning(AiRRobot robot) {
        aiRRobot=robot;
        svet1 = aiRRobot.linearOpMode.hardwareMap.dcMotor.get("svet1");
        svet2 = aiRRobot.linearOpMode.hardwareMap.dcMotor.get("svet2");
        svet1.setDirection(DcMotorSimple.Direction.FORWARD);
        svet2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPower(double x) {
        svet1.setPower(x);
        svet2.setPower(x);
    }

    public void smooth() {
        double t = System.currentTimeMillis() / 1000.0;
        setPower((Math.sin(t) + 1) / 2);
    }

    public void update() {
        switch (lightMode) {
            case ON:
                setPower(1);
                break;
            case OFF:
                setPower(0);
                break;
            case SMOOTH:
                smooth();
                break;
        }
    }
}

