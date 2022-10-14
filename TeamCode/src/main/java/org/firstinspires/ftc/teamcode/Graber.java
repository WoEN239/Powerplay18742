package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Graber {
    Servo servo;

    public Graber(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "Servo");
    }

    void Target_Graber(boolean triang) {
        boolean oldtriang = false;
        double triang_angle = 0;
        if (triang == true && oldtriang == false && triang_angle != 0) {
            servo.setPosition(0);
            triang_angle = 0;
        } else {
            if (triang == true && oldtriang == false && triang_angle != 0.25) {
                servo.setPosition(0.25);
                triang_angle = 0.25;
            }
        }
    }
}

