package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
@Autonomous
public class Program_Avtonom1 extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

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

        //D(50);
        //sleep(100);
        //D(-50);
        //sleep(100);
        RL(50);
        sleep(100);
        RL(-50);
        sleep(100);
        B(50);
        sleep(100);
        B(-50);
        sleep(100);
        M(50);
        sleep(100);
        M(-50);
        sleep(100);
    }

    void RL(double x) {
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

            motors = (-lfd - lbd + rfd + rbd) / 4;
            e = x * crr - motors;

            double k = 0.001;

            leftFrontDrive.setPower(-e * k);
            rightFrontDrive.setPower(e * k);
            leftBackDrive.setPower(-e * k);
            rightBackDrive.setPower(e * k);
            telemetry.addData("lfd", lfd);
            telemetry.addData("lbd", lbd);
            telemetry.addData("rfd", rfd);
            telemetry.addData("rbd", rbd);
            telemetry.addData("e", e);
            telemetry.update();

        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void B(double x) {
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

            motors = (-lfd + lbd + rfd - rbd) / 4;
            e = x * crr - motors;

            double k = 0.001;

            leftFrontDrive.setPower(-e * k);
            rightFrontDrive.setPower(e * k);
            leftBackDrive.setPower(e * k);
            rightBackDrive.setPower(-e * k);
            telemetry.addData("lfd", lfd);
            telemetry.addData("lbd", lbd);
            telemetry.addData("rfd", rfd);
            telemetry.addData("rbd", rbd);
            telemetry.addData("e", e);
            telemetry.update();

        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    void M(double x) {
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

            double k = 0.001;

            leftFrontDrive.setPower(e * k);
            rightFrontDrive.setPower(e * k);
            leftBackDrive.setPower(e * k);
            rightBackDrive.setPower(e * k);
            telemetry.addData("lfd", lfd);
            telemetry.addData("lbd", lbd);
            telemetry.addData("rfd", rfd);
            telemetry.addData("rbd", rbd);
            telemetry.addData("e", e);
            telemetry.update();

        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    void D(double x) {
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

            double k = 0.001;
            double k2 = 2;

            leftFrontDrive.setPower(e * k * k2);
            rightFrontDrive.setPower(e * k);
            leftBackDrive.setPower(e * k);
            rightBackDrive.setPower(e * k * k2);
            telemetry.addData("lfd", lfd);
            telemetry.addData("lbd", lbd);
            telemetry.addData("rfd", rfd);
            telemetry.addData("rbd", rbd);
            telemetry.addData("e", e);
            telemetry.update();

        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}