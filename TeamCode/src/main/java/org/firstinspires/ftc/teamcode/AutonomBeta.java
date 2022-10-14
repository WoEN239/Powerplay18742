package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutonomBeta extends LinearOpMode {
    DriveTrain driveTrain;
    Graber graber;
 public void runOpMode() {
     driveTrain=new DriveTrain(hardwareMap);
     graber=new Graber(hardwareMap);
     waitForStart();
     driveTrain.setMotor(100,0,0);
     graber.Target_Graber(true);
 }
}