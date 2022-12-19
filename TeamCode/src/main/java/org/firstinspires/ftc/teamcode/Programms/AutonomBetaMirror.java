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
public class AutonomBetaMirror extends LinearOpMode {
    AiRRobot aiRRobot;
    Camera camera;
    public void runOpMode() {
        aiRRobot = new AiRRobot(this);
        camera = new Camera(hardwareMap);
        waitForStart();
        aiRRobot.lift.reset();;
        // aiRRobot.graber.Target_Graber(true);
        aiRRobot.lightning.lightMode= Lightning.LightningMode.SMOOTH;
        aiRRobot.lightning.update();
        aiRRobot.graber.Target_Graber(true);
        sleep(500);
        aiRRobot.lift.setMotor(Lift.LiftPosition.LOW);
        int  c = camera.readCamera();
        telemetry.addData("camera", c);
        telemetry.update();
        camera.stopcamera();
        aiRRobot.driveTrain.setFieldPosition(72.195,1.4,0);
         aiRRobot.driveTrain.setFieldPosition(72.195,15,57);
        sleep(500);
        aiRRobot.lift.setMotor(Lift.LiftPosition.MIDDLE);
        aiRRobot.driveTrain.setFieldPosition(87.101,15,57);
        aiRRobot.graber.Target_Graber(false);
        sleep(500);
        aiRRobot.driveTrain.setFieldPosition(72,0,57);
        aiRRobot.driveTrain.setFieldPosition(72,0,0);
        sleep(500);
        aiRRobot.graber.Target_Graber(true);
        //aiRRobot.driveTrain.setFieldPosition(56,0,29);
        //aiRRobot.driveTrain.setFieldPosition(56,0,0);

        if(c == 18){
            aiRRobot.driveTrain.setFieldPosition(60,100,0);
            aiRRobot.driveTrain.setFieldPosition(60,100,0);
            aiRRobot.driveTrain.setFieldPosition(70,100,0);
        }
        if (c == 6){
            aiRRobot.driveTrain.setFieldPosition(60,-100,0);
            aiRRobot.driveTrain.setFieldPosition(60,-100,0);
            aiRRobot.driveTrain.setFieldPosition(70,-100,0);
        }
    }


}
