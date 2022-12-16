package org.firstinspires.ftc.teamcode.Programms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.AiRRobot;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Graber;
import org.firstinspires.ftc.teamcode.Robot.Lift;
import org.firstinspires.ftc.teamcode.Robot.Lightning;
import org.firstinspires.ftc.teamcode.Robot.Odometry;

@TeleOp
public class TeleOpM extends LinearOpMode {
    boolean oldsquare = false;
    boolean graberPosition;
    boolean oldcircle = false;
    boolean oldtriangle = false;
    public double speed = 1.0;

    double liftpos = 0;

    AiRRobot aiRRobot;

    @Override
    public void runOpMode() throws InterruptedException {

        aiRRobot = new AiRRobot(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        aiRRobot.lightning.lightMode = Lightning.LightningMode.SMOOTH;
        aiRRobot.lift.liftPosition = Lift.LiftPosition.ZERO;
        while (opModeIsActive()) {
            boolean square = gamepad1.square;
            if (square && !oldsquare) {
                graberPosition = !graberPosition;
            }


            boolean circle = gamepad1.circle;
            aiRRobot.graber.Target_Graber(graberPosition);
            boolean triangle = gamepad1.triangle;
            boolean cross = gamepad1.cross;
            /*if (gamepad1.dpad_down) {
                aiRRobot.lift.liftMode = Lift.LiftMode.AUTO;
                aiRRobot.lift.liftPosition = Lift.LiftPosition.ZERO;
            }
            if (gamepad1.dpad_up) {
                aiRRobot.lift.liftMode = Lift.LiftMode.AUTO;
                aiRRobot.lift.liftPosition = Lift.LiftPosition.UP;
            }
            if (gamepad1.dpad_left) {
                aiRRobot.lift.liftMode = Lift.LiftMode.AUTO;
                aiRRobot.lift.liftPosition = Lift.LiftPosition.LOW;
            }
            if (gamepad1.dpad_right) {
                aiRRobot.lift.liftMode = Lift.LiftMode.AUTO;
                aiRRobot.lift.liftPosition = Lift.LiftPosition.MIDDLE;
            }

             */
            if (gamepad1.left_trigger > 0.1) {
                aiRRobot.lift.liftMode = Lift.LiftMode.MANUAl;
            } else if (triangle || cross) {
                aiRRobot.lift.liftMode = Lift.LiftMode.MANUALLIMIT;
            }
            if (triangle) {
                aiRRobot.lift.power = 0.6;
            }
            if (cross) {
                aiRRobot.lift.power = -0.15;
            }
            if (!triangle && !cross) {
                aiRRobot.lift.power = 0.1;
            }
            aiRRobot.odometry.update();
            aiRRobot.lift.update();


            aiRRobot.odometry.update();
           telemetry.addData("x", aiRRobot.odometry.x);
            telemetry.addData("y", aiRRobot.odometry.y);
            telemetry.addData("heading", aiRRobot.odometry.heading);
            aiRRobot.lightning.update();

            double axial = -gamepad1.left_stick_y * speed;
            double lateral = -gamepad1.left_stick_x * speed;
            double yaw = -gamepad1.right_stick_x * speed;

            if (gamepad1.right_trigger > 0.1 || (aiRRobot.lift.encoders() > 130 && axial < 0)) {
                if(aiRRobot.lift.encoders() > 1600 && axial < 0) {
                    axial /= 4.5;
                    lateral /= 4.5;
                    yaw /= 4.5;
                } else {
                    axial /= 3.5;
                    lateral /= 3.5;
                    yaw /= 3.5;
                }

            }


            aiRRobot.driveTrain.setPowers(axial, lateral, yaw);
            oldsquare = square;
            oldcircle = circle;
            oldtriangle = triangle;
            //driveTrain.positionsEncodersXY();
            telemetry.update();
        }
    }
}
