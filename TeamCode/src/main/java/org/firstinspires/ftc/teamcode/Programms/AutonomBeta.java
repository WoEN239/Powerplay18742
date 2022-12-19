package org.firstinspires.ftc.teamcode.Programms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.AiRRobot;
import org.firstinspires.ftc.teamcode.Robot.Camera;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Graber;
import org.firstinspires.ftc.teamcode.Robot.Lift;
import org.firstinspires.ftc.teamcode.Robot.Lightning;

@Autonomous
public class AutonomBeta extends LinearOpMode {
    AiRRobot aiRRobot;
Camera camera;
    public void runOpMode() {
        aiRRobot = new AiRRobot(this);
        camera = new Camera(hardwareMap);
        waitForStart();
        int  c = camera.readCamera();
        telemetry.addData("camera", c);
        telemetry.update();
        camera.stopcamera();
       // aiRRobot.graber.Target_Graber(true);
        aiRRobot.lightning.lightMode= Lightning.LightningMode.SMOOTH;
        aiRRobot.lightning.update();
        aiRRobot.driveTrain.setFieldPosition(124,0,0);
        aiRRobot.driveTrain.setFieldPosition(124,0,-29);
        aiRRobot.driveTrain.setFieldPosition(129,0,-29);
        aiRRobot.driveTrain.setFieldPosition(124,0,0);
        aiRRobot.driveTrain.setFieldPosition(124,0,90);
        for (int i=0;i<1;i++) {
            aiRRobot.driveTrain.setFieldPosition(124, 81, 90);
            aiRRobot.driveTrain.setFieldPosition(124, 91, 90);
            aiRRobot.driveTrain.setFieldPosition(124, -55, 90);
            aiRRobot.driveTrain.setFieldPosition(124, -55, 71);
            aiRRobot.driveTrain.setFieldPosition(124, -60, 71);
            aiRRobot.driveTrain.setFieldPosition(124, -55, 71);
            aiRRobot.driveTrain.setFieldPosition(124, -55, 71);
        }
        if(c == 18){
            aiRRobot.driveTrain.setFieldPosition(164,0,90);
            aiRRobot.driveTrain.setFieldPosition(164,0,180);
        }
        if(c == 0){
            aiRRobot.driveTrain.setFieldPosition(90,0,90);
            aiRRobot.driveTrain.setFieldPosition(90,0,180);
        }
        if (c == 6){
            aiRRobot.driveTrain.setFieldPosition(55,0,180)  ;
        }
        //aiRRobot.driveTrain.setFieldPosition(124,-4,0);
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
