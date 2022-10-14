package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutonomBeta extends LinearOpMode {
    DriveTrain driveTrain;
 public void runOpMode() {
     driveTrain=new DriveTrain(hardwareMap);
     waitForStart();
     driveTrain.setMotor(100,0,0);
 }
}