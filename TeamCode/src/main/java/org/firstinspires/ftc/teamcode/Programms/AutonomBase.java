package org.firstinspires.ftc.teamcode.Programms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.AiRRobot;


abstract public class AutonomBase extends LinearOpMode {
    AiRRobot aiRRobot;

    @Override
    public void runOpMode() {
        aiRRobot = new AiRRobot(this);
        waitForStart();
        main();
    }

    abstract public void main();
}
