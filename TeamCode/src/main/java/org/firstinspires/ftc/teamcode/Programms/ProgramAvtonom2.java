package org.firstinspires.ftc.teamcode.Programms;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;

public class ProgramAvtonom2 extends LinearOpMode {
    BNO055IMU gyro;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor svet1 = null;
    private DcMotor svet2 = null;
    double crr = 24 * 20 / (9.8 * PI);
    @Override
    public void runOpMode() {
        gyro = hardwareMap.get(BNO055IMU.class, "imu");

        gyro.initialize(new BNO055IMU.Parameters());

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        svet1 = hardwareMap.get(DcMotor.class, "svet1");
        svet2 = hardwareMap.get(DcMotor.class, "svet2");

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
        TurnGuro(85, 0.20);
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
            double k = 0.09;
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
    void TurnGuro(double x, double v) {
        double angle = toDegrees(gyro.getAngularOrientation().firstAngle);
        while ((abs(angle)) < x && opModeIsActive()){
            leftFrontDrive.setPower(-v);
            rightFrontDrive.setPower(v);
            leftBackDrive.setPower(-v);
            rightBackDrive.setPower(v);
            svet1.setPower(0);
            svet1.setPower(0);
            angle = toDegrees(gyro.getAngularOrientation().firstAngle);
            telemetry.addData("angle", angle);
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
            double k = 0.01;
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
            double k = 0.01;
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
        double ex = x * crr - motorsX;
        double ey = y * crr - motorsY;
        double ez = z * crr - motorsZ;
        while ((abs(ex)) > 100 && (abs(ey)) > 100 && (abs(z)) > 100 && opModeIsActive()) {
            lfd = leftFrontDrive.getCurrentPosition();
            lbd = leftBackDrive.getCurrentPosition();
            rfd = rightFrontDrive.getCurrentPosition();
            rbd = rightBackDrive.getCurrentPosition();
            motorsX = (lfd + lbd + rfd + rbd) / 4;
            motorsY = (-lfd + lbd + rfd - rbd) / 4;
            motorsZ = (-lfd - lbd + rfd + rbd) / 4;
            ex = x * crr - motorsX;
            ey = y * crr - motorsY;
            ez = z * crr - motorsZ;
            double kx = 0.01;
            double ky = 0.01;
            double kz = 0.01;
            leftFrontDrive.setPower(ex * kx - ey * ky - ez * kz);
            rightFrontDrive.setPower(ex * kx + ey * ky - ez * kz);
            leftBackDrive.setPower(ex * kx + ey * ky + ez * kz);
            rightBackDrive.setPower(ex * kx - ey * ky + ez * kz);
            telemetry.addData("lfd", lfd);
            telemetry.addData("lbd", lbd);
            telemetry.addData("rfd", rfd);
            telemetry.addData("rbd", rbd);
            telemetry.addData("ex", ex);
            telemetry.addData("ey", ey);
            telemetry.addData("ez", ez);
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