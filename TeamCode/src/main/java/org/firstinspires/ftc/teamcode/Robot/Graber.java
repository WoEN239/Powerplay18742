package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Graber {
    Servo servo;
    AiRRobot aiRRobot;
    boolean pos = false;

    double timeGraber = System.currentTimeMillis() / 1000;

    public Graber(AiRRobot robot) {
        aiRRobot = robot;
        servo = aiRRobot.linearOpMode.hardwareMap.get(Servo.class, "Servo");
    }

    public void Target_Graber(boolean triang) {
        if (pos != triang) {
            timeGraber = System.currentTimeMillis();
        }
        if (triang) {
            servo.setPosition(0.759);
        } else {
            servo.setPosition(0.97);

        }
        pos = triang;
    }

    public boolean getPosition() {
        if (System.currentTimeMillis() < timeGraber + 300) {
            return !pos;
        } else {
            return pos;
        }
    }

}