package org.firstinspires.ftc.teamcode.Programms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.AiRRobot;
import org.firstinspires.ftc.teamcode.Robot.Camera;
import org.firstinspires.ftc.teamcode.Robot.Lift;

@Autonomous
public class AutonomBetaMirror extends LinearOpMode {
    AiRRobot aiRRobot;
    Camera camera;

    public void runOpMode() {
        aiRRobot = new AiRRobot(this);
        camera = new Camera(hardwareMap);
        aiRRobot.lift.reset();
        waitForStart();
        int  c = camera.readCamera();
        telemetry.addData("camera", c);
        telemetry.update();
        camera.stopcamera();
        aiRRobot.graber.Target_Graber(true);
        aiRRobot.driveTrain.setFieldPosition(124,0,0);
        aiRRobot.driveTrain.setFieldPosition(124,0,-40);
        aiRRobot.lift.setMotor(Lift.LiftPosition.UP);
        aiRRobot.driveTrain.setMotor3axes(25,0,0);
        aiRRobot.graber.Target_Graber(false);
        aiRRobot.driveTrain.setMotor3axes(-25,0,0);
        aiRRobot.driveTrain.setFieldPosition(124,0,85);
        aiRRobot.driveTrain.setFieldPosition(150,0,85);
        aiRRobot.lift.setMotor(Lift.LiftPosition.LOW);
        aiRRobot.graber.Target_Graber(true);
        aiRRobot.lift.setMotor(Lift.LiftPosition.MIDDLE);
        aiRRobot.driveTrain.setFieldPosition(124,0,-40);
       /* driveTrain = new DriveTrain(hardwareMap, this);
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


        driveTrain.setMotor3axes(-10,0,0);
        if(c==18){
            driveTrain.setMotor3axes(0,0,90);
            driveTrain.setMotor3axes(60,0,0);
        }
        if(c==6){
            driveTrain.setMotor3axes(0,0,-90);
            driveTrain.setMotor3axes(60,0,0);
        }


        */




    }

}
