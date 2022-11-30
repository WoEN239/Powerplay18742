package org.firstinspires.ftc.teamcode.Programms;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.Robot.DriveTrain;


public class ProgramAvtonom1 extends LinearOpMode {
    DriveTrain driveTrain;
    double crr = 24 * 20 / (9.8 * PI);

    @Override
    public void runOpMode() {

        driveTrain=new DriveTrain(hardwareMap, this);
        driveTrain.reset();
        waitForStart();
        /*   driveTrain.displayEncoders();
            telemetry.update();
        }

         */
        driveTrain.setMotor3axes(0,90,0);
        driveTrain.setMotor3axes(0,-90,0);


    }
}