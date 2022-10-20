package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class Program2  extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor svet = null;
    Servo servo;
    boolean oldsquare = false;
    double square_angle = 0;
    double circle_angle = 0;
    boolean oldcircle = false;
    boolean oldtriangle = false;
    double triangle_angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        svet = hardwareMap.get(DcMotor.class, "svet");
        servo = hardwareMap.get(Servo.class, "Servo");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        svet.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            boolean square = gamepad1.square;
            if (square == true && oldsquare == false && square_angle != 0) {
                servo.setPosition(0);
                square_angle = 0;
            } else {
                if (square == true && oldsquare == false && square_angle != 0.25) {
                    servo.setPosition(1);
                    square_angle = 1;
                }
            }
            boolean circle = gamepad1.circle;
            if (circle == true && oldcircle == false && circle_angle != 0) {
                leftFrontDrive.setPower(0);
                circle_angle = 0;
            } else {
                if (circle == true && oldcircle == false && circle_angle != 0.25) {
                    leftFrontDrive.setPower(0);
                    circle_angle = 1;
                }
            }
            boolean triangle = gamepad1.triangle;
            if (triangle == true && oldtriangle == false && triangle_angle != 0) {
                leftFrontDrive.setPower(0);
                triangle_angle = 0;
            } else {
                if (triangle == true && oldtriangle == false && triangle_angle != 0.25) {
                    leftFrontDrive.setPower(0);
                    triangle_angle = 1;
                }
            }
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            oldsquare = square;
            oldcircle = circle;
            oldtriangle = triangle;
            }

        }
    }
