package org.firstinspires.ftc.teamcode.Robot;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Odometry {
    BNO055IMU gyro;

    double crr = 24 * 20 / (9.8 * PI)/(124/121.8);

    private DcMotor left_front_drive;
    private DcMotor left_back_drive;
    private DcMotor right_front_drive;
    private DcMotor right_back_drive;
    public double x = 0, y = 0, heading = 0;

    double lfdold = 0;
    double lbdold = 0;
    double rfdold = 0;
    double rbdold = 0;
    LinearOpMode linearOpMode1;
    AiRRobot aiRRobot;
    public Odometry(AiRRobot robot) {
        aiRRobot=robot;
        gyro = aiRRobot.linearOpMode.hardwareMap.get(BNO055IMU.class, "imu");

        left_front_drive = aiRRobot.linearOpMode.hardwareMap.dcMotor.get("left_front_drive");
        left_back_drive = aiRRobot.linearOpMode.hardwareMap.dcMotor.get("left_back_drive");
        right_front_drive = aiRRobot.linearOpMode.hardwareMap.dcMotor.get("right_front_drive");
        right_back_drive = aiRRobot.linearOpMode.hardwareMap.dcMotor.get("right_back_drive");

        left_front_drive.setDirection(DcMotor.Direction.FORWARD);
        left_back_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);


    }

    public void update() {
        int lfd = left_front_drive.getCurrentPosition();
        int lbd = left_back_drive.getCurrentPosition();
        int rfd = right_front_drive.getCurrentPosition();
        int rbd = right_back_drive.getCurrentPosition();
        double angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;

        double deltaX = xPosition(lfd - lfdold, lbd - lbdold, rfd - rfdold, rbd - rbdold);
        double deltaY = yPosition(lfd - lfdold, lbd - lbdold, rfd - rfdold, rbd - rbdold);
        deltaY= deltaY*0.85602812451;
        x += deltaX * cos(toRadians(-angle)) + deltaY * sin(toRadians(-angle));
        y += -deltaX * sin(toRadians(-angle)) + deltaY * cos(toRadians(-angle));
        heading = angle;
        lfdold = lfd;
        lbdold = lbd;
        rfdold = rfd;
        rbdold = rbd;
    }

    public double xPosition(double lfd, double lbd, double rfd, double rbd) {
        double x = ((lfd + lbd + rfd + rbd) / 4) / crr;
        return x;
    }

    public double yPosition(double lfd, double lbd, double rfd, double rbd) {
        double y = ((-lfd + lbd + rfd - rbd) / 4) / crr;
        return y;
    }
}
