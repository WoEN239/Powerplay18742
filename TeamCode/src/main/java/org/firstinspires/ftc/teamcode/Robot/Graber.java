package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Graber {
    Servo servo;

    public Graber(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "Servo");
    }

    public void Target_Graber(boolean triang) {

        if (triang) {
            servo.setPosition(0.805);
        }
        else {
            servo.setPosition(1);

        }
    }
}

