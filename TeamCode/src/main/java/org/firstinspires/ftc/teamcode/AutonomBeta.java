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
        driveTrain.TurnGuro(90,0.15);
        driveTrain.Horizontal(50);
        lift.setMotor(1);
    }

}
