package org.firstinspires.ftc.teamcode.Programms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Camera;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Graber;
import org.firstinspires.ftc.teamcode.Robot.Lift;
import org.firstinspires.ftc.teamcode.Robot.Lightning;

@Autonomous
public class AutonomBetaMirror extends LinearOpMode {
    DriveTrain driveTrain;
    Graber graber;
    Lift lift;
    Camera camera;
    Lightning lightning;

    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap, this);
        graber = new Graber(hardwareMap);
        lift = new Lift(hardwareMap, this);
        lightning = new Lightning(hardwareMap);
        camera = new Camera(hardwareMap);
        lift.reset();
        waitForStart();
        lift.reset();
        int  c = camera.readCamera();
        telemetry.addData("camera", c);
        telemetry.update();
        camera.stopcamera();
        graber.Target_Graber(true);
        sleep(500);
        driveTrain.setMotor3axes(60,0,0);
        driveTrain.setMotor3axes(0, 0, 40);
        lift.setMotor(Lift.LiftPosition.UP);
        driveTrain.setMotor3axes(25, 0, 0);
        sleep(500);
        graber.Target_Graber(false);
        sleep(500);
        driveTrain.setMotor3axes(-25, 0, 0);
        lift.setMotor(Lift.LiftPosition.ZERO);
        sleep(500);
        driveTrain.setMotor3axes(0, 0, 40);/*
        driveTrain.setMotor3axes(0, 0, 180);
        driveTrain.setMotor3axes(60, 0, 0);
        driveTrain.setMotor3axes(0, 0, 90);
        driveTrain.setMotor3axes(45, 0, 0);
        graber.Target_Graber(true);
        sleep(500);
        driveTrain.setMotor3axes(-45,0,0);
        driveTrain.setMotor3axes(0,0,-90);
        driveTrain.setMotor3axes(60, 0, 0);
        driveTrain.setMotor3axes(0, 0, -30);
        lift.setMotor(Lift.LiftPosition.UP);
        driveTrain.setMotor3axes(25, 0, 0);
        graber.Target_Graber(false);
        sleep(500);
        driveTrain.setMotor3axes(-25, 0, 0);
        driveTrain.setMotor3axes(0, 0, 30);

        */
        driveTrain.setMotor3axes(-10,0,0);
        if(c==18){
            driveTrain.setMotor3axes(0,0,90);
            driveTrain.setMotor3axes(60,0,0);
        }
        if(c==6){
            driveTrain.setMotor3axes(0,0,-90);
            driveTrain.setMotor3axes(60,0,0);
        }





    }

}