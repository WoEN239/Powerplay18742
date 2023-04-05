package org.firstinspires.ftc.teamcode.Programms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.AiRRobot;
import org.firstinspires.ftc.teamcode.Robot.Camera;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Graber;
import org.firstinspires.ftc.teamcode.Robot.Lift;
import org.firstinspires.ftc.teamcode.Robot.Lightning;

@Autonomous
public class AutonoTest extends LinearOpMode {

    AiRRobot aiRRobot;

    public void runOpMode() {
       aiRRobot = new AiRRobot(this);
       aiRRobot.lift.reset();
        waitForStart();
        aiRRobot.lift.setMotor(Lift.LiftPosition.UP);
        aiRRobot.lift.setMotor(Lift.LiftPosition.ZERO);
        /*lift.reset();
        sleep(500);
        lift.setMotor(Lift.LiftPosition.UP);
        sleep(500);
        lift.setMotor(Lift.LiftPosition.ZERO);
        sleep(500);


         */




    }

}
