package org.firstinspires.ftc.teamcode.Programms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.AiRRobot;
import org.firstinspires.ftc.teamcode.Robot.Camera;
import org.firstinspires.ftc.teamcode.Robot.Lift;

@Autonomous
public class AutonomBetaUniversal extends LinearOpMode {
    AiRRobot aiRRobot;
    Camera camera;

    public void runOpMode() {
        aiRRobot = new AiRRobot(this);
        camera = new Camera(hardwareMap);
        aiRRobot.lift.reset();
        waitForStart();
        aiRRobot.lift.setMotor(Lift.LiftPosition.LOW);
        int c = camera.readCamera();
        telemetry.addData("camera", c);
        telemetry.update();
        camera.stopcamera();
        aiRRobot.graber.Target_Graber(false);
        aiRRobot.driveTrain.setFieldPosition(60,0,0);
        if(c == 18){
            aiRRobot.driveTrain.setFieldPosition(60,0,-90);
            aiRRobot.driveTrain.setFieldPosition(60,55,-90);
            aiRRobot.driveTrain.setFieldPosition(60,55,-180);
        }
        if (c == 6){
            aiRRobot.driveTrain.setFieldPosition(60,0,90);
            aiRRobot.driveTrain.setFieldPosition(60,-55,90);
            aiRRobot.driveTrain.setFieldPosition(60,-55,180);
        }
        if(c == 0){
            aiRRobot.driveTrain.setFieldPosition(60,0,90);
        }
    }
}
