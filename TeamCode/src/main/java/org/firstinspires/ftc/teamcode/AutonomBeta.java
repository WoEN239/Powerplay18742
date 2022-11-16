package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        int  c = camera.readCamera();
        telemetry.addData("camera", c);
        telemetry.update();

        waitForStart();
        graber.Target_Graber(false);
        driveTrain.setMotor3axes(0,0,180);
        driveTrain.setMotor3axes(0, 45, 0);
        driveTrain.setMotor3axes(45, 0, 0);
        driveTrain.setMotor3axes(0, 0, 45);
        lift.setMotor(Lift.LiftPosition.UP);
        driveTrain.setMotor3axes(10, 0, 0);
        graber.Target_Graber(true);
        driveTrain.setMotor3axes(-10, 0, 0);
        lift.setMotor(Lift.LiftPosition.ZERO);
        driveTrain.setMotor3axes(0, 0, -45);
        driveTrain.setMotor3axes(0, 0, 180);
        driveTrain.setMotor3axes(55, 0, 0);
        driveTrain.setMotor3axes(0, 0, -90);
        driveTrain.setMotor3axes(45, 0, 0);
        graber.Target_Graber(false);
        driveTrain.setMotor3axes(-45,0,0);
        driveTrain.setMotor3axes(0,0,-90);
        driveTrain.setMotor3axes(45, 0, 0);
        driveTrain.setMotor3axes(0, 0, 45);
        lift.setMotor(Lift.LiftPosition.UP);
        driveTrain.setMotor3axes(10, 0, 0);
        graber.Target_Graber(true);
        driveTrain.setMotor3axes(-10, 0, 0);
        driveTrain.setMotor3axes(0, 0, -45);
        if(c==0){
            driveTrain.setMotor3axes(0,-270,0);
        }
        if(c==6){
            driveTrain.setMotor3axes(0,-180,0);
        }
        if(c==18){
            driveTrain.setMotor3axes(0,-90,0);
        }
    }

}
