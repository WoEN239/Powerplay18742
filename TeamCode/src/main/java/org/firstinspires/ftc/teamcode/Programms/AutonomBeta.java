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
public class AutonomBeta extends LinearOpMode {
      AiRRobot aiRRobot;

    public void runOpMode() {

       /* lift.reset();
        waitForStart();
        lift.reset();
        int  c = camera.readCamera();
        telemetry.addData("camera", c);
        telemetry.update();
        camera.stopcamera();
        graber.Target_Graber(true);
        sleep(500);
        driveTrain.setMotor3axes(62,0,0);
        driveTrain.setMotor3axes(0, 0, -38);
        lift.setMotor(Lift.LiftPosition.UP);
        driveTrain.setMotor3axes(25, 0, 0);
        sleep(2000);
        graber.Target_Graber(false);
        sleep(500);
        driveTrain.setMotor3axes(-25, 0, 0);
        lift.setMotor(Lift.LiftPosition.ZERO);
        sleep(500);
        driveTrain.setMotor3axes(0, 0, 38);
        /*
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
            driveTrain.setMotor3axes(53,0,0);
            driveTrain.setMotor3axes(0,0,-90);
        }
        if(c==6){
            driveTrain.setMotor3axes(0,0,-90);
            driveTrain.setMotor3axes(53,0,0);
            driveTrain.setMotor3axes(0,0,90);
        }
      while (opModeIsActive())
          lightning.smooth();



*/
    }



}
