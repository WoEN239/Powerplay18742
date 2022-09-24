package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import static java.lang.Math.abs;
@Autonomous
@Disabled
public class Program_Avtonom2 extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    Servo servo;

    double crr = 24 * 20;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        servo = hardwareMap.get(Servo.class, "Servo");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        Move(100);
    }

    void Move(double x) {
        double lfd = leftFrontDrive.getCurrentPosition();
        double lbd = leftBackDrive.getCurrentPosition();
        double rfd = rightFrontDrive.getCurrentPosition();
        double rbd = rightBackDrive.getCurrentPosition();

        double motors = (lfd + lbd + rfd + rbd) / 4;
        while ((abs(x * crr)) > (abs(motors)) && opModeIsActive()) {
            lfd = leftFrontDrive.getCurrentPosition();
            lbd = leftBackDrive.getCurrentPosition();
            rfd = rightFrontDrive.getCurrentPosition();
            rbd = rightBackDrive.getCurrentPosition();

            motors = (lfd + lbd + rfd + rbd) / 4;

            double e = x * crr - motors;
            double k = 0.01;

            leftFrontDrive.setPower(e * k);
            rightFrontDrive.setPower(e * k);
            leftBackDrive.setPower(e * k);
            rightBackDrive.setPower(e * k);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
