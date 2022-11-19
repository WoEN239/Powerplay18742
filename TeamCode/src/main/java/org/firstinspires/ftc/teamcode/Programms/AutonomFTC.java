package org.firstinspires.ftc.teamcode.Programms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Camera;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Graber;
import org.firstinspires.ftc.teamcode.Robot.Lift;


public class AutonomFTC extends LinearOpMode {
    DriveTrain driveTrain;
    Graber graber;
    Lift lift;
    Camera camera;

    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap, this);
        graber = new Graber(hardwareMap);
        lift = new Lift(hardwareMap, this );
        camera = new Camera(hardwareMap);
        waitForStart();
        int  c = camera.readCamera();
        telemetry.addData("camera", c);
        telemetry.update();
        graber.Target_Graber(false);
         //driveTrain.setMotor(100, 50, 0);
         sleep(100);
         driveTrain.setMotor3axes(0, 50, 0);
        sleep(100);
          driveTrain.setMotor3axes(0,0,50);
        sleep(100);
        sleep(500);
        //lift.setMotor(3);
        graber.Target_Graber(true);
    }
}