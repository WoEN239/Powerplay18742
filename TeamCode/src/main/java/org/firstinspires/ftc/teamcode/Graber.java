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
        double angel = 0;
        if (triang == true) {
            angel = 0.25;
        }
        if (triang == true && oldtriang == false && triang_angle != 0.13) {
            servo.setPosition(0.13);
            triang_angle = 0.13;
        } else {
            if (triang == true && oldtriang == false && triang_angle != 0.5) {
                servo.setPosition(0.5);
                triang_angle = 0.5;
            }
        }
    }
}

