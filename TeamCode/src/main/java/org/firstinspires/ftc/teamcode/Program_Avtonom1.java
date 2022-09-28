package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
@Autonomous
//@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
public class Program_Avtonom1 extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    double crr = 24 * 20 / (9.8 * PI);


    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        //Left(-50);
        Left(10);
    }

    void Left(double y) {
        double llfd = leftFrontDrive.getCurrentPosition();
        double llbd = leftBackDrive.getCurrentPosition();
        double lrfd = rightFrontDrive.getCurrentPosition();
        double lrbd = rightBackDrive.getCurrentPosition();

        double motors2 = (llfd + llbd + lrfd + lrbd) / 4;
        double e2 = y * crr - motors2;
        while ((abs(e2)) > 1 && opModeIsActive()) {
            llfd = leftFrontDrive.getCurrentPosition();
            llbd = leftBackDrive.getCurrentPosition();
            lrfd = rightFrontDrive.getCurrentPosition();
            lrbd = rightBackDrive.getCurrentPosition();

            motors2 = (llfd + llbd + lrfd + lrbd) / 4;
            e2 = y * crr - motors2;

            double k = 0.00001;

            leftFrontDrive.setPower(-e2 * k);
            rightFrontDrive.setPower(e2 * k);
            leftBackDrive.setPower(-e2 * k);
            rightBackDrive.setPower(e2 * k);
            telemetry.addData("llfd", llfd);
            telemetry.addData("llbd", llbd);
            telemetry.addData("lrfd", lrfd);
            telemetry.addData("lrbd", lrbd);
            telemetry.addData("e2", e2);
            telemetry.update();

        }
    }
    void Move(double x) {
        double lfd = leftFrontDrive.getCurrentPosition();
        double lbd = leftBackDrive.getCurrentPosition();
        double rfd = rightFrontDrive.getCurrentPosition();
        double rbd = rightBackDrive.getCurrentPosition();

        double motors = (lfd + lbd + rfd + rbd) / 4;
        double e = x * crr - motors;
        while ((abs(e)) > 100 && opModeIsActive()) {
            lfd = leftFrontDrive.getCurrentPosition();
            lbd = leftBackDrive.getCurrentPosition();
            rfd = rightFrontDrive.getCurrentPosition();
            rbd = rightBackDrive.getCurrentPosition();

            motors = (lfd + lbd + rfd + rbd) / 4;
            e = x * crr - motors;

            double k = 0.01;

            leftFrontDrive.setPower(e * k);
            rightFrontDrive.setPower(e * k);
            leftBackDrive.setPower(e * k);
            rightBackDrive.setPower(e * k);
            telemetry.addData("loop count", motors);
            telemetry.addData("e", e);
            telemetry.update();

        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
