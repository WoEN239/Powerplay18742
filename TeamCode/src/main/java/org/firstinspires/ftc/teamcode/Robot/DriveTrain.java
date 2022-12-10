package org.firstinspires.ftc.teamcode.Robot;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class DriveTrain {
    BNO055IMU gyro;
    private DcMotor left_front_drive;
    private DcMotor left_back_drive;
    private DcMotor right_front_drive;
    private DcMotor right_back_drive;
    private PidRegulator PIDX = new PidRegulator(0.015, 0.0000001, 0.0000001);
    private PidRegulator PIDY = new PidRegulator(0.025, 0.0000001, 0.0000001);
    private PidRegulator PIDZ = new PidRegulator(0.015, 0, 0);
    private PidRegulator PIDFIELDX = new PidRegulator(0.015, 0.0000001, 0.0000001);
    private PidRegulator PIDFIELDY = new PidRegulator(0.015, 0.0000001, 0.0000001);
    double told;
    double crr = 24 * 20 / (9.8 * PI);
    private LinearOpMode opMode;
    double targetangle = 0;
    double xold = 0, yold = 0;
    AiRRobot aiRRobot;

    public DriveTrain(AiRRobot robot) {
        aiRRobot = robot;
        gyro = robot.linearOpMode.hardwareMap.get(BNO055IMU.class, "imu");

        gyro.initialize(new BNO055IMU.Parameters());
        left_front_drive = robot.linearOpMode.hardwareMap.dcMotor.get("left_front_drive");
        left_back_drive = robot.linearOpMode.hardwareMap.dcMotor.get("left_back_drive");
        right_front_drive = robot.linearOpMode.hardwareMap.dcMotor.get("right_front_drive");
        right_back_drive = robot.linearOpMode.hardwareMap.dcMotor.get("right_back_drive");
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_back_drive.setDirection(DcMotor.Direction.REVERSE);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_back_drive.setDirection(DcMotor.Direction.FORWARD);
        left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        reset();
    }

    public void reset() {
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double xPosition() {
        double lfd = left_front_drive.getCurrentPosition();
        double lbd = left_back_drive.getCurrentPosition();
        double rfd = right_front_drive.getCurrentPosition();
        double rbd = right_back_drive.getCurrentPosition();
        double x = ((lfd + lbd + rfd + rbd) / 4) / crr;
        return x;
    }

    public double yPosition() {
        double lfd = left_front_drive.getCurrentPosition();
        double lbd = left_back_drive.getCurrentPosition();
        double rfd = right_front_drive.getCurrentPosition();
        double rbd = right_back_drive.getCurrentPosition();
        double y = ((-lfd + lbd + rfd - rbd) / 4) / crr;
        return y;
    }

    public void positionsEncodersXY() {
        double x = xPosition();
        double y = yPosition();
        opMode.telemetry.addData("x", x);
        opMode.telemetry.addData("y", y);
    }

    public void setPowers(double x, double y, double z) {
        x = Range.clip(x, -1, 1);
        y = Range.clip(y, -1, 1);
        z = Range.clip(z, -1, 1);
        double leftFrontMotorPower = x - y - z;
        double rightFrontMotorPower = x + y + z;
        double leftRearMotorPower = x + y - z;
        double rightRearMotorPower = x - y + z;
        left_front_drive.setPower(leftFrontMotorPower);
        left_back_drive.setPower(leftRearMotorPower);
        right_front_drive.setPower(rightFrontMotorPower);
        right_back_drive.setPower(rightRearMotorPower);
    }

    public void displayEncoders() {
        opMode.telemetry.addData("lfd", left_front_drive.getCurrentPosition());
        opMode.telemetry.addData("lrd", left_back_drive.getCurrentPosition());
        opMode.telemetry.addData("rfd", right_front_drive.getCurrentPosition());
        opMode.telemetry.addData("rrd", right_back_drive.getCurrentPosition());
    }


    public void setMotor3axes(double x, double y, double z) {
        reset();

        double lfd = left_front_drive.getCurrentPosition();
        double lbd = left_back_drive.getCurrentPosition();
        double rfd = right_front_drive.getCurrentPosition();
        double rbd = right_back_drive.getCurrentPosition();

        double motorsX = (lfd + lbd + rfd + rbd) / 4;
        double motorsY = (-lfd + lbd + rfd - rbd) / 4;

        double targetx = x * crr;
        double targety = y * crr;
        double errx = targetx - motorsX;
        double erry = targety - motorsY;
        targetangle = targetangle + z;
        double errz = targetangle - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        while (abs(errz) > 180) {
            errz -= 360 * signum(errz);

        }
        PIDX.update(errx);
        PIDY.update(erry);
        PIDZ.update(errz);
        double t1 = System.currentTimeMillis() / 1000.0;
        double t = 0;
        double tr = t - told;

        while (((abs(errx)) > 75 || (abs(erry)) > 75 || (abs(errz)) > 4) && tr < 5 && aiRRobot.linearOpMode.opModeIsActive()) {
            t = System.currentTimeMillis() / 1000.0;
            tr = t - t1;
            lfd = left_front_drive.getCurrentPosition();
            lbd = left_back_drive.getCurrentPosition();
            rfd = right_front_drive.getCurrentPosition();
            rbd = right_back_drive.getCurrentPosition();


            double angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            motorsX = (lfd + lbd + rfd + rbd) / 4;
            motorsY = (-lfd + lbd + rfd - rbd) / 4;
            //motorsZ = (-lfd - lbd + rfd + rbd) / 4;
            targetx = x * crr;
            targety = y * crr;
            errx = targetx - motorsX;
            erry = targety - motorsY;
            errz = targetangle - angle;
            while (abs(errz) > 180) {
                errz -= 360 * signum(errz);

            }

            opMode.telemetry.addData("angel", angle);
            opMode.telemetry.update();

            double powerx = PIDX.update(errx);
            double powery = PIDY.update(erry);
            double powerz = PIDZ.update(errz);


            setPowers(Range.clip(powerx, -0.4, 0.4), powery, powerz);
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

    public void setFieldPosition(double x, double y, double heading) {

        double errx = aiRRobot.odometry.x;
        double erry = aiRRobot.odometry.y;
        targetangle = targetangle + heading;
        double errz = targetangle - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        while (abs(errz) > 180) {
            errz -= 360 * signum(errz);

        }
        PIDX.update(errx);
        PIDY.update(erry);
        PIDZ.update(errz);
        double t1 = System.currentTimeMillis() / 1000.0;
        double t = 0;
        double tr = t - told;

        while (((abs(errx)) > 3 || (abs(erry)) > 3 || (abs(errz)) > 4) && tr < 5 && aiRRobot.linearOpMode.opModeIsActive()) {
            t = System.currentTimeMillis() / 1000.0;
            tr = t - t1;


            errx = aiRRobot.odometry.x;
            erry = aiRRobot.odometry.y;
            errz = targetangle - aiRRobot.odometry.heading;
            while (abs(errz) > 180) {
                errz -= 360 * signum(errz);

            }

           aiRRobot.odometry.update();

            double powerx = PIDFIELDX.update(errx);
            double powery = PIDFIELDY.update(erry);
            double powerz = PIDZ.update(errz);

            setPowersField(Range.clip(powerx, -0.4, 0.4), powery, powerz);
            told = t;

        }
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

    }


    public void setPowersField(double x, double y, double heading) {
        double angle = aiRRobot.odometry.heading;
        double powersX = x * cos(toRadians(-angle)) + y * sin(toRadians(-angle));
        double powersY = -x * sin(toRadians(-angle)) + y * cos(toRadians(-angle));
        setPowers(powersX, powersY, heading);
    }
}

