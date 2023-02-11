package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Graber {
    Servo servo;
    Servo servo1;
    AiRRobot aiRRobot;
    boolean pos = false;

    double timeGraber = System.currentTimeMillis() / 1000;

    public Graber(AiRRobot robot) {
        aiRRobot = robot;
        servo = aiRRobot.linearOpMode.hardwareMap.get(Servo.class, "Servo1");
        servo1 = aiRRobot.linearOpMode.hardwareMap.get(Servo.class, "Servo");
    }

    public static double POS_CLOSE = 0.75;
    public static double POS_OPEN = 1.00;

    public void Target_Graber(boolean triang) {
        if (pos != triang)
            timeGraber = System.currentTimeMillis();
        if (triang)
            servo.setPosition(POS_CLOSE);
        else
            servo.setPosition(POS_OPEN);
        pos = triang;
    }


    public boolean getPosition() {
        if (System.currentTimeMillis() < timeGraber + 300)
            return !pos;
        else
            return pos;
    }

}