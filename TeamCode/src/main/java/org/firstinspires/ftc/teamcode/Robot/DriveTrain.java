package org.firstinspires.ftc.teamcode.Robot;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Config
public class DriveTrain {
    BNO055IMU gyro;
    private DcMotor left_front_drive;
    private DcMotor left_back_drive;
    private DcMotor right_front_drive;
    private DcMotor right_back_drive;
    private PidRegulator PIDX = new PidRegulator(kPdrive, kIdrive, kDdrive);
    private PidRegulator PIDY = new PidRegulator(kPdrive, kIdrive, kDdrive);
    private PidRegulator PIDZ = new PidRegulator(kProtation, kIrotation, kDrotation);
    private PidRegulator PIDFIELDX = new PidRegulator(kPdrive, kIdrive, kDdrive);
    private PidRegulator PIDFIELDY = new PidRegulator(kPdrive, kIdrive, kDdrive);
    public static double kPdrive = 0.05;
    public static double kIdrive = 0.055;
    public static double kDdrive = 0;
    public static double kProtation = 0.04;
    public static double kIrotation = 0.01;
    public static double kDrotation = 0;
    double told;
    double crr = 24 * 20 / (9.8 * PI);
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
        left_front_drive.setDirection(DcMotor.Direction.FORWARD);
        left_back_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);
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
    }

    public void setPowers(double x, double y, double z) {
        y *= 1.2;
        double leftFrontMotorPower = x - y - z;
        double rightFrontMotorPower = x + y + z;
        double leftRearMotorPower = x + y - z;
        double rightRearMotorPower = x - y + z;
        double biggestPower = 0;

        if (abs(leftFrontMotorPower) > 1 || abs(leftRearMotorPower) > 1 || abs(rightFrontMotorPower) > 1 || abs(rightRearMotorPower) > 1) {
            biggestPower = max(max(abs(leftFrontMotorPower), abs(leftRearMotorPower)), max(abs(rightFrontMotorPower), abs(rightRearMotorPower)));
            leftFrontMotorPower /= biggestPower;
            leftRearMotorPower /= biggestPower;
            rightFrontMotorPower /= biggestPower;
            rightRearMotorPower /= biggestPower;
        }

        left_front_drive.setPower(leftFrontMotorPower);
        left_back_drive.setPower(leftRearMotorPower);
        right_front_drive.setPower(rightFrontMotorPower);
        right_back_drive.setPower(rightRearMotorPower);
    }

  /*  public void displayEncoders() {
        opMode.telemetry.addData("lfd", left_front_drive.getCurrentPosition());
        opMode.telemetry.addData("lrd", left_back_drive.getCurrentPosition());
        opMode.telemetry.addData("rfd", right_front_drive.getCurrentPosition());
        opMode.telemetry.addData("rrd", right_back_drive.getCurrentPosition());

    }
   */


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


            double powerx = PIDX.update(errx);
            double powery = PIDY.update(erry);
            double powerz = PIDZ.update(errz);

            if (t < 0.5) {
                powerx = t/500*powerx;
                powery = t/500*powery;
                powerz = t=500*powerz;
            }
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
        PIDFIELDX.reset();
        PIDFIELDY.reset();
        PIDZ.reset();
        double errx = x - aiRRobot.odometry.x;
        double erry = y - aiRRobot.odometry.y;
        targetangle = heading;
        double errz = targetangle - aiRRobot.odometry.heading;

        while (abs(errz) > 180) {
            errz -= 360 * signum(errz);

        }
        PIDFIELDX.update(errx);
        PIDFIELDY.update(erry);
        PIDZ.update(errz);
        double t1 = System.currentTimeMillis() / 1000.0;
        double t = 0;
        double tr = t - told;

        while (((abs(errx)) > 3 || (abs(erry)) > 3 || (abs(errz)) > 4) && tr < 5 && aiRRobot.linearOpMode.opModeIsActive()) {
            t = System.currentTimeMillis() / 1000.0;
            tr = t - t1;

            errx = x - aiRRobot.odometry.x;
            erry = y - aiRRobot.odometry.y;
            errz = targetangle - aiRRobot.odometry.heading;
            while (abs(errz) > 180) {
                errz -= 360 * signum(errz);

            }

            aiRRobot.odometry.update();

            double powerx = PIDFIELDX.update(errx);
            double powery = PIDFIELDY.update(erry);
            double powerz = PIDZ.update(errz);
            if (t < 0.5) {
                powerx = t/500*powerx;
                powery = t/500*powery;
                powerz = t=500*powerz;
            }
            setPowersField(Range.clip(powerx, -0.8, 0.8), Range.clip(powery, -0.8, 0.8), Range.clip(powerz, -0.8, 0.8));
            told = t;

        }
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

    }


    public void setPowersField(double x, double y, double heading) {
        double angle = aiRRobot.odometry.heading;
        double powersX = x * cos(toRadians(angle)) + y * sin(toRadians(angle));
        double powersY = -x * sin(toRadians(angle)) + y * cos(toRadians(angle));
        setPowers(powersX, powersY, heading);
    }
}

