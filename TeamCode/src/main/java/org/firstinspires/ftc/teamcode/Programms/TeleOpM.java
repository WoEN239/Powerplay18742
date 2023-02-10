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

    boolean oldbumer=false;
    AiRRobot aiRRobot;
    boolean speedcontrol=false;

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
            if(gamepad1.ps){
                aiRRobot.lift.reset();
            }


            boolean circle = gamepad1.circle;
            aiRRobot.graber.Target_Graber(graberPosition);
            boolean triangle = gamepad1.triangle;
            boolean cross = gamepad1.cross;
            if (gamepad1.dpad_down) {
                aiRRobot.lift.liftMode = Lift.LiftMode.AUTO;
                aiRRobot.lift.liftPosition = Lift.LiftPosition.ZERO;
            }

            /*if (gamepad1.dpad_up) {
                aiRRobot.lift.liftMode = Lift.LiftMode.AUTO;
                aiRRobot.lift.liftPosition = Lift.LiftPosition.UP;
            }
            */
            if (gamepad1.dpad_left) {
                aiRRobot.lift.liftMode = Lift.LiftMode.AUTO;
                aiRRobot.lift.liftPosition = Lift.LiftPosition.LOW;
            }
            if (gamepad1.dpad_right) {
                aiRRobot.lift.liftMode = Lift.LiftMode.AUTO;
                aiRRobot.lift.liftPosition = Lift.LiftPosition.MIDDLE;
            }

            if (gamepad1.left_trigger > 0.1) {
                aiRRobot.lift.liftMode = Lift.LiftMode.MANUAl;
            } else if (triangle || cross) {
                aiRRobot.lift.liftMode = Lift.LiftMode.MANUALLIMIT;
            }
            if (triangle) {
                aiRRobot.lift.power = 1.0;
            }
            if (cross) {
                aiRRobot.lift.power = -0.1;
            }
            if (!triangle && !cross) {
                aiRRobot.lift.power = 0.1;
            }
            aiRRobot.odometry.update();
            aiRRobot.lift.update();


            telemetry.addData("x", aiRRobot.odometry.x);
            telemetry.addData("y", aiRRobot.odometry.y);
            telemetry.addData("heading", aiRRobot.odometry.heading);
            telemetry.addData("motor1", aiRRobot.lift.motor1.getCurrentPosition());
            telemetry.addData("motor2",aiRRobot.lift.motor2.getCurrentPosition());
            telemetry.addData("top",aiRRobot.lift.buttonUp.getState());
            telemetry.addData("down",aiRRobot.lift.buttonDown.getState());
            aiRRobot.lightning.update();
              aiRRobot.driveTrain.displayEncoders();
            double axial = -gamepad1.left_stick_y * speed;
            double lateral = -gamepad1.left_stick_x * speed;
            double yaw = -gamepad1.right_stick_x * speed;

            if(gamepad1.right_bumper) {
                axial /= 3;
                lateral /= 3;
                yaw /= 3;
            }
            oldbumer=gamepad1.right_bumper;
            aiRRobot.driveTrain.setPowers(axial, lateral, yaw);
            oldsquare = square;
            oldcircle = circle;
            oldtriangle = triangle;
            //driveTrain.positionsEncodersXY();
            telemetry.update();
        }
    }
}
