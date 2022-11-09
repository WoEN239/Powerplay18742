package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutonomBeta extends LinearOpMode {
    DriveTrain driveTrain;
    Graber graber;
    Lift lift;
    Camera camera;
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap, this);
        graber = new Graber(hardwareMap);
        lift = new Lift(hardwareMap, this );
        /*camera = new Camera(hardwareMap);
        int  c = camera.readCamera();
        telemetry.addData("camera", c);
        telemetry.update();

         */
        driveTrain.setMotor3axes(90,0,0);
        driveTrain.setMotor3axes(-90,0,0);
        driveTrain.setMotor3axes(0,90,0);
        driveTrain.setMotor3axes(90,-90,0);
        driveTrain.setMotor3axes(0,90,0);
        driveTrain.setMotor3axes(90,-90,0);
        driveTrain.Horizontal(50);
    }

}
