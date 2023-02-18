package org.firstinspires.ftc.teamcode.Programms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.AiRRobot;
import org.firstinspires.ftc.teamcode.Robot.Camera;
import org.firstinspires.ftc.teamcode.Robot.Lift;

@Autonomous
public class AutonomBetaMirror extends LinearOpMode {
    AiRRobot aiRRobot;
    Camera camera;

    public void runOpMode() {
        aiRRobot = new AiRRobot(this);
        camera = new Camera(hardwareMap);
        waitForStart();
        aiRRobot.lift.reset();
        int c = camera.readCamera();
        telemetry.addData("camera", c);
        telemetry.update();
        camera.stopcamera();
        aiRRobot.graber.Target_Graber(true);
        sleep(500);
        aiRRobot.driveTrain.setFieldPosition(102, 1.4, 0);
        aiRRobot.lift.setMotor(Lift.LiftPosition.UP);
        aiRRobot.driveTrain.setFieldPosition(141.39, -21.9, -29.5);
        aiRRobot.graber.Target_Graber(false);
        sleep(500);
        aiRRobot.driveTrain.setFieldPosition(136.39, -16.9, -29.5);
        aiRRobot.lift.setMotor(Lift.LiftPosition.ZERO);
        aiRRobot.driveTrain.setFieldPosition(130, 0, 0);
        /*aiRRobot.driveTrain.setFieldPosition(120, 1.4, 0);
        aiRRobot.lift.setMotor(Lift.LiftPosition.CUPSON1);
        aiRRobot.driveTrain.setFieldPosition(120, 91.4, -90);
        aiRRobot.graber.Target_Graber(true);
        sleep(500);
        aiRRobot.lift.setMotor(Lift.LiftPosition.UP);
        aiRRobot.driveTrain.setFieldPosition(137.97, 22.2, 25.06);
        aiRRobot.graber.Target_Graber(false);
        sleep(500);
        aiRRobot.lift.setMotor(Lift.LiftPosition.ZERO);
        aiRRobot.driveTrain.setFieldPosition(120, 1.4, 0);
        aiRRobot.lift.setMotor(Lift.LiftPosition.CUPSON2);
        aiRRobot.driveTrain.setFieldPosition(120, 91.4, -90);
        aiRRobot.graber.Target_Graber(true);
        sleep(500);
        aiRRobot.lift.setMotor(Lift.LiftPosition.UP);
        aiRRobot.driveTrain.setFieldPosition(137.97, 22.2, 25.06);
        aiRRobot.graber.Target_Graber(false);
        sleep(500);
        aiRRobot.lift.setMotor(Lift.LiftPosition.ZERO);

         */
        if (c == 18) {
            aiRRobot.driveTrain.setFieldPosition(130, 0, -90);
            aiRRobot.driveTrain.setFieldPosition(130, 55, -90);

        }
        if (c == 6) {
            aiRRobot.driveTrain.setFieldPosition(130, 0, 90);
            aiRRobot.driveTrain.setFieldPosition(130, -60, 90);
        }
        if (c == 0) {
            aiRRobot.driveTrain.setFieldPosition(130, 5, 0);
        }
    }


}
