package org.firstinspires.ftc.teamcode.Programms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Camera;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Graber;
import org.firstinspires.ftc.teamcode.Robot.Lift;
import org.firstinspires.ftc.teamcode.Robot.Lightning;

@Autonomous
public class AutonoTest extends LinearOpMode {

    Lift lift;
    Lightning lightning;

    public void runOpMode() {
        lift = new Lift(hardwareMap, this);
        lightning = new Lightning(hardwareMap);
        lift.reset();
        waitForStart();
        lift.reset();
        sleep(500);
        lift.setMotor(Lift.LiftPosition.UP);
        sleep(500);
        lift.setMotor(Lift.LiftPosition.ZERO);
        sleep(500);





    }

}
