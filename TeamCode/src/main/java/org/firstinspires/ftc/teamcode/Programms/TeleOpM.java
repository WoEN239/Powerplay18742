package org.firstinspires.ftc.teamcode.Programms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    DriveTrain driveTrain;
    Graber graber;
    Lift lift;
    Lightning lightning;
    Odometry odometry;
    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap, this);
        /*graber = new Graber(hardwareMap);
        lightning = new Lightning(hardwareMap);
        lift = new Lift(hardwareMap, this);

         */
        odometry=new Odometry(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
       /* lightning.lightMode = Lightning.LightningMode.SMOOTH;
        lift.liftPosition = Lift.LiftPosition.ZERO;

        */
        while (opModeIsActive()) {
            boolean square = gamepad1.square;
            /*if (square && !oldsquare) {
                graberPosition = !graberPosition;
            }

             */
            boolean circle = gamepad1.circle;
           // graber.Target_Graber(graberPosition);
            boolean triangle = gamepad1.triangle;
            boolean cross = gamepad1.cross;
          /*  if(gamepad1.dpad_down) {
                lift.liftMode = Lift.LiftMode.AUTO;
                lift.liftPosition = Lift.LiftPosition.ZERO;
            }
            if(gamepad1.dpad_up) {
                lift.liftMode = Lift.LiftMode.AUTO;
                lift.liftPosition = Lift.LiftPosition.UP;
            }
            if(gamepad1.dpad_left) {
                lift.liftMode = Lift.LiftMode.AUTO;
                lift.liftPosition = Lift.LiftPosition.LOW;
            }
            if(gamepad1.dpad_right) {
                lift.liftMode = Lift.LiftMode.AUTO;
                lift.liftPosition = Lift.LiftPosition.MIDDLE;
            }
            if (gamepad1.left_trigger > 0.1) {
                lift.liftMode = Lift.LiftMode.MANUAl;
            } else if (triangle || cross) {
                lift.liftMode = Lift.LiftMode.MANUALLIMIT;
            }
            if (triangle) {
                lift.power = 1;
            }
            if (cross) {
                lift.power = -1;
            }
            if (!triangle && !cross) {
                lift.power = 0;
            }
            odometry.update();
            lift.update();

           */
            odometry.update();
            telemetry.addData("x",odometry.x);
            telemetry.addData("y",odometry.y);
            driveTrain.displayEncoders();
            telemetry.addData("heading",odometry.heading);

           // lightning.update();

            double axial = -gamepad1.left_stick_y * speed;
            double lateral = -gamepad1.left_stick_x * speed;
            double yaw = -gamepad1.right_stick_x * speed;


            /*if (gamepad1.right_trigger > 0.1 || (lift.encoders() > 130 && axial < 0)) {
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


             */
            driveTrain.setPowers(axial, lateral, yaw);
            oldsquare = square;
            oldcircle = circle;
            oldtriangle = triangle;
            //driveTrain.positionsEncodersXY();
            telemetry.update();
        }
        }
    }
