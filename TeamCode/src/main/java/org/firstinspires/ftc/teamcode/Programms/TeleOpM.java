package org.firstinspires.ftc.teamcode.Programms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Graber;
import org.firstinspires.ftc.teamcode.Robot.Lift;
import org.firstinspires.ftc.teamcode.Robot.Lightning;

@TeleOp
public class TeleOpM extends LinearOpMode {
    boolean oldsquare = false;
    boolean graberPosition;
    boolean oldcircle = false;
    boolean oldtriangle = false;
    public double speed = 1.0;

    double liftpos = 0;

    DriveTrain driveTrain;
    Graber graber;
    Lift lift;
    Lightning lightning;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap, this);
        graber = new Graber(hardwareMap);
        lightning = new Lightning(hardwareMap);
        lift = new Lift(hardwareMap, this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            boolean square = gamepad1.square;
            if (square && !oldsquare) {
                graberPosition = !graberPosition;
            }
            boolean circle = gamepad1.circle;
            graber.Target_Graber(graberPosition);
            boolean triangle = gamepad1.triangle;
            boolean cross = gamepad1.cross;
            //if (gamepad1.touchpad_finger_1) {
            //if() {
            //    double c=lift.setMotorToZero(Lift.LiftPosition.ZERO);
            //}
            //}
            if (gamepad1.left_trigger > 0.1) {
                if (triangle) {
                    lift.setPowers(1);
                    lightning.smooth();
                    liftpos = 1;
                }
                if (cross) {
                    lift.setPowers(-1);
                    lightning.smooth();
                    liftpos = -1;
                }
                if (!triangle && !cross) {
                    lift.setPowers(0);
                    lightning.smooth();
                    liftpos = 0;
                }
            } else {
                if (triangle) {
                    lift.setPowersLimit(1);
                    lightning.smooth();
                }
                if (cross) {
                    lift.setPowersLimit(-1);
                    lightning.smooth();
                }
                if (!triangle && !cross) {
                    lift.setPowersLimit(0);
                    lightning.smooth();
                }
                double axial = -gamepad1.left_stick_y * speed;
                double lateral = -gamepad1.left_stick_x * speed;
                double yaw = -gamepad1.right_stick_x * speed;


                if (gamepad1.right_trigger > 0.1 || (lift.encoders() > 130 && axial < 0)) {
                    if (lift.encoders() > 700 && axial < 0) {
                        axial /= 4.5;
                        lateral /= 4.5;
                        yaw /= 4.5;
                    } else {
                        axial /= 3.5;
                        lateral /= 3.5;
                        yaw /= 3.5;
                    }

                }

                driveTrain.setPowers(axial, lateral, yaw);
                oldsquare = square;
                oldcircle = circle;
                oldtriangle = triangle;
                driveTrain.positionsEncodersXY();
                telemetry.update();
            }
        }
    }
}
