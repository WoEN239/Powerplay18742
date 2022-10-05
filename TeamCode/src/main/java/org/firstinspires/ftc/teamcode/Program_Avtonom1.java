package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
@Autonomous
public class Program_Avtonom1 extends LinearOpMode {
    private PidRegulator PIDX=new PidRegulator(0.025,0.0000001,0.001);
    private PidRegulator PIDY=new PidRegulator(0.025,0.0000001,0.001);
    private PidRegulator PIDZ=new PidRegulator(0.025,0.0000001,0.001);
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

        Diogonal(50, 50, 50);
        sleep(100);
        Diogonal(-50, -50, 50);

    }
    void Turn(double x) {
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
    void LeftRight(double x) {
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
    void ForwardBack(double x) {
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
    void Diogonal(double x, double y, double z) {
        double lfd = leftFrontDrive.getCurrentPosition();
        double lbd = leftBackDrive.getCurrentPosition();
        double rfd = rightFrontDrive.getCurrentPosition();
        double rbd = rightBackDrive.getCurrentPosition();

        double motorsX = (lfd + lbd + rfd + rbd) / 4;
        double motorsY = (-lfd + lbd + rfd - rbd) / 4;
        double motorsZ = (-lfd - lbd + rfd + rbd) / 4;

        PIDX.target = x * crr;
        PIDY.target = y * crr;
        PIDZ.target = z * crr ;

        PIDX.update(motorsX);
        PIDY.update(motorsY);
        PIDZ.update(motorsZ);

        while ((abs( PIDX.err)) > 100 && (abs(PIDY.err)) > 100 && (abs(PIDZ.err)) > 100 && opModeIsActive()) {
            lfd = leftFrontDrive.getCurrentPosition();
            lbd = leftBackDrive.getCurrentPosition();
            rfd = rightFrontDrive.getCurrentPosition();
            rbd = rightBackDrive.getCurrentPosition();

            motorsX = (lfd + lbd + rfd + rbd) / 4;
            motorsY = (-lfd + lbd + rfd - rbd) / 4;
            motorsZ = (-lfd - lbd + rfd + rbd) / 4;
            double powerx = PIDX.update(motorsX);
            double powery =  PIDY.update(motorsY);
            double powerz = PIDZ.update(motorsZ);

            double kx = 0.001;
            double ky = 0.001;
            double kz = 0.01;

            leftFrontDrive.setPower(powerx - powery - powerz);
            rightFrontDrive.setPower(powerx + powery - powerz);
            leftBackDrive.setPower(powerx + powery - powerz);
            rightBackDrive.setPower(powerx - powery + powerz);
            telemetry.addData("lfd", lfd);
            telemetry.addData("lbd", lbd);
            telemetry.addData("rfd", rfd);
            telemetry.addData("rbd", rbd);
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