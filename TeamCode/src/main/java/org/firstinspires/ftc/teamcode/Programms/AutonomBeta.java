package org.firstinspires.ftc.teamcode.Programms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Camera;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Graber;
import org.firstinspires.ftc.teamcode.Robot.Lift;
import org.firstinspires.ftc.teamcode.Robot.Lightning;

@Autonomous
public class AutonomBeta extends LinearOpMode {
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
        lift.setMotor(Lift.LiftPosition.MIDDLE);
        lift.setMotor(Lift.LiftPosition.GROUND);
        /*graber.Target_Graber(true);
        sleep(500);
        driveTrain.setMotor3axes(80,0,0);
        driveTrain.setMotor3axes(0, 0, -45);
        lift.setMotor(Lift.LiftPosition.UP);
        driveTrain.setMotor3axes(30, 0, 0);
        graber.Target_Graber(false);
        sleep(500);
        driveTrain.setMotor3axes(-30, 0, 0);
        lift.setMotor(Lift.LiftPosition.ZERO);
        driveTrain.setMotor3axes(0, 0, 45);
        driveTrain.setMotor3axes(0, 0, 180);
        driveTrain.setMotor3axes(55, 0, 0);
        driveTrain.setMotor3axes(0, 0, -90);
        driveTrain.setMotor3axes(45, 0, 0);
        graber.Target_Graber(true);
        sleep(500);
        driveTrain.setMotor3axes(-45,0,0);
        driveTrain.setMotor3axes(0,0,-90);
        driveTrain.setMotor3axes(45, 0, 0);
        driveTrain.setMotor3axes(0, 0, -45);
        lift.setMotor(Lift.LiftPosition.UP);
        driveTrain.setMotor3axes(10, 0, 0);
        graber.Target_Graber(false);
        sleep(500);
        driveTrain.setMotor3axes(-10, 0, 0);
        driveTrain.setMotor3axes(0, 0, 45);
       /* if(c==18){
            driveTrain.setMotor3axes(0,-270,0);
        }
        if(c==6){
            driveTrain.setMotor3axes(0,-90,0);
        }



         */
    }

}
