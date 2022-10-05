package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutonomBeta extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private PidRegulator PID=new PidRegulator(0.025,0.0000001,0.001);
    public void runOpMode() throws InterruptedException {
        waitForStart();
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PID.target = 90*4;
        while (opModeIsActive()) {
            double power= PID.update(leftFrontDrive.getCurrentPosition());
            leftFrontDrive.setPower(power);
            telemetry.addData("target-current position",PID.target- leftFrontDrive.getCurrentPosition());
            telemetry.addData("power",power);
            telemetry.update();
        }
    }
}
