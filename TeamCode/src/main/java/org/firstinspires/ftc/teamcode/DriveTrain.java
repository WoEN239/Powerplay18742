package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class DriveTrain {
    BNO055IMU gyro;
    private DcMotor left_front_drive;
    private DcMotor left_back_drive;
    private DcMotor right_front_drive;
    private DcMotor right_back_drive;
    private PidRegulator PIDX = new PidRegulator(0.025, 0.0000001, 0);
    private PidRegulator PIDY = new PidRegulator(0.025, 0.0000001, 0);
    private PidRegulator PIDZ = new PidRegulator(0.0025, 0, 0);
    double told;
    double crr = 24 * 20 / (9.8 * PI);
    private LinearOpMode opMode;

    public DriveTrain(HardwareMap hardwareMap, LinearOpMode _opMode) {
        opMode = _opMode;
        gyro = hardwareMap.get(BNO055IMU.class, "imu");

        gyro.initialize(new BNO055IMU.Parameters());
        left_front_drive = hardwareMap.dcMotor.get("left_front_drive");
        left_back_drive = hardwareMap.dcMotor.get("left_back_drive");
        right_front_drive = hardwareMap.dcMotor.get("right_front_drive");
        right_back_drive = hardwareMap.dcMotor.get("right_back_drive");
        left_front_drive.setDirection(DcMotor.Direction.FORWARD);
        left_back_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);
        left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    void reset() {

        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void setPowers(double x, double y, double z) {
        double leftFrontMotorPower = x - y - z;
        double rightFrontMotorPower = x + y + z;
        double leftRearMotorPower = x + y - z;
        double rightRearMotorPower = x - y + z;
        left_front_drive.setPower(leftFrontMotorPower);
        left_back_drive.setPower(leftRearMotorPower);
        right_front_drive.setPower(rightFrontMotorPower);
        right_back_drive.setPower(rightRearMotorPower);
    }

    void displayEncoders() {
        opMode.telemetry.addData("lfd",  left_front_drive.getCurrentPosition());
        opMode.telemetry.addData("lrd",  left_back_drive.getCurrentPosition());
        opMode.telemetry.addData("rfd",  right_front_drive.getCurrentPosition());
        opMode.telemetry.addData("rrd", right_back_drive.getCurrentPosition());
    }



    void setMotor3axes(double x, double y, double z) {
        reset();
        double lfd = left_front_drive.getCurrentPosition();
        double lbd = left_back_drive.getCurrentPosition();
        double rfd = right_front_drive.getCurrentPosition();
        double rbd = right_back_drive.getCurrentPosition();

        double motorsX = (lfd + lbd + rfd + rbd) / 4;
        double motorsY = (-lfd + lbd + rfd - rbd) / 4;
        double motorsZ = (-lfd - lbd + rfd + rbd) / 4;

        double targetx = x * crr;
        double targety = y * crr;
        double targetz = z;
        double errx = targetx - motorsX;
        double erry = targety - motorsY;
        double errz = targetz - motorsZ;

        PIDX.update(errx);
        PIDY.update(erry);
        PIDZ.update(errz);
        double t1 = System.currentTimeMillis() / 1000.0;
        double t = 0;
        double tr = t - told;
        while (((abs(errx)) > 100 || (abs(erry)) > 100 || (abs(errz)) > 100) && tr < 5000 && opMode.opModeIsActive()) {
            t = System.currentTimeMillis() / 1000.0;
            tr = t - t1;
            lfd = left_front_drive.getCurrentPosition();
            lbd = left_back_drive.getCurrentPosition();
            rfd = right_front_drive.getCurrentPosition();
            rbd = right_back_drive.getCurrentPosition();

            //TODO remove toDegrees()
            double angle = toDegrees(gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);

            motorsX = (lfd + lbd + rfd + rbd) / 4;
            motorsY = (-lfd + lbd + rfd - rbd) / 4;
            //motorsZ = (-lfd - lbd + rfd + rbd) / 4;
            targetx = x * crr;
            targety = y * crr;
            targetz = z;
            errx = targetx - motorsX;
            erry = targety - motorsY;
            errz = targetz - angle;
            opMode.telemetry.addData("angel", errz);
            opMode.telemetry.update();
            if (errz > 180) {
                errz -= 360;
            }
            if (errz < -180) {
                errz += 360;
            }

            double powerx = PIDX.update(errx);
            double powery = PIDY.update(erry);
            double powerz = PIDZ.update(errz);


            left_front_drive.setPower(powerx - powery - powerz);
            left_back_drive.setPower(powerx + powery - powerz);
            right_front_drive.setPower(powerx + powery + powerz);
            right_back_drive.setPower(powerx - powery + powerz);
            told = t;

        }
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void Horizontal(double x) {
        double lfd = left_front_drive.getCurrentPosition();
        double lbd = left_back_drive.getCurrentPosition();
        double rfd = right_front_drive.getCurrentPosition();
        double rbd = right_back_drive.getCurrentPosition();
        double motors = (lfd + lbd + rfd + rbd) / 4;
        double e = x * crr - motors;
        while ((abs(e)) > 100 && opMode.opModeIsActive()) {
            lfd = left_front_drive.getCurrentPosition();
            lbd = left_back_drive.getCurrentPosition();
            rfd = right_front_drive.getCurrentPosition();
            rbd = right_back_drive.getCurrentPosition();
            motors = (-lfd - lbd + rfd + rbd) / 4;
            e = x * crr - motors;
            double k = 0.09;
            left_front_drive.setPower(e * k);
            right_front_drive.setPower(-e * k);
            left_front_drive.setPower(e * k);
            right_back_drive.setPower(-e * k);
            /*telemetry.addData("lfd", lfd);
            telemetry.addData("lbd", lbd);
            telemetry.addData("rfd", rfd);
            telemetry.addData("rbd", rbd);
            telemetry.addData("e", e);
            telemetry.update();

             */
        }
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void Move(double x) {
        double lfd = left_front_drive.getCurrentPosition();
        double lbd = left_back_drive.getCurrentPosition();
        double rfd = right_front_drive.getCurrentPosition();
        double rbd = right_back_drive.getCurrentPosition();
        double motors = (lfd + lbd + rfd + rbd) / 4;
        double e = x * crr - motors;
        while ((abs(e)) > 100 && opMode.opModeIsActive()) {
            lfd = left_front_drive.getCurrentPosition();
            lbd = left_back_drive.getCurrentPosition();
            rfd = right_front_drive.getCurrentPosition();
            rbd = right_back_drive.getCurrentPosition();
            motors = (lfd + lbd + rfd + rbd) / 4;
            e = x * crr - motors;
            double k = 0.01;
            left_front_drive.setPower(e * k);
            right_front_drive.setPower(e * k);
            left_back_drive.setPower(e * k);
            right_back_drive.setPower(e * k);
        }
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}

