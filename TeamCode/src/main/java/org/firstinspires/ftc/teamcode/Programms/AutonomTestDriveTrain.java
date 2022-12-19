package org.firstinspires.ftc.teamcode.Programms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutonomTestDriveTrain extends AutonomBase{
    @Override
    public void main() {
       // aiRRobot.driveTrain.setFieldPosition(10,0,0);
        //aiRRobot.driveTrain.setFieldPosition(10,0,90);
        //aiRRobot.driveTrain.setFieldPosition(10,0,0);
        aiRRobot.driveTrain.setFieldPosition(0,50,0);
        aiRRobot.driveTrain.setFieldPosition(0,0,0);
    }
}
