package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.*;

public class DriveTrain {
    private DcMotor left_front_drive;
    private DcMotor left_back_drive;
    private DcMotor right_front_drive;
    private DcMotor right_back_drive;
    private PidRegulator PIDX = new PidRegulator(0.025, 0.0000001, 0.001);
    private PidRegulator PIDY = new PidRegulator(0.025, 0.0000001, 0.001);
    private PidRegulator PIDZ = new PidRegulator(0.025, 0.0000001, 0.001);
    double told;
    double crr = 24 * 20 / (9.8 * PI);

    public DriveTrain(HardwareMap hardwareMap) {
        left_front_drive = hardwareMap.dcMotor.get("left_front_drive");
        left_back_drive = hardwareMap.dcMotor.get("left_back_drive");
        right_front_drive = hardwareMap.dcMotor.get("right_front_drive");
        right_back_drive = hardwareMap.dcMotor.get("right_back_drive");
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
        double rightFrontMotorPower = x + y - z;
        double leftRearMotorPower = x + y - z;
        double rightRearMotorPower = x - y + z;
        left_front_drive.setPower(leftFrontMotorPower);
        left_back_drive.setPower(rightFrontMotorPower);
        right_front_drive.setPower(leftRearMotorPower);
        right_back_drive.setPower(rightRearMotorPower);
    }

    void setMotor(double x, double y, double z) {
        double lfd = left_front_drive.getCurrentPosition();
        double lbd = left_back_drive.getCurrentPosition();
        double rfd = right_front_drive.getCurrentPosition();
        double rbd = right_back_drive.getCurrentPosition();

        double motorsX = (lfd + lbd + rfd + rbd) / 4;
        double motorsY = (-lfd + lbd + rfd - rbd) / 4;
        double motorsZ = (-lfd - lbd + rfd + rbd) / 4;

        PIDX.target = x * crr;
        PIDY.target = y * crr;
        PIDZ.target = z * crr;

        PIDX.update(motorsX);
        PIDY.update(motorsY);
        PIDZ.update(motorsZ);
        double t1 = System.currentTimeMillis() / 1000.0;
        double t = 0;
        double tr = t - told;
        while (((abs(PIDX.err)) > 100 || (abs(PIDY.err)) > 100 ||  (abs(PIDZ.err)) > 100) && tr < 5000) {
            t = System.currentTimeMillis() / 1000.0;
            tr = t - t1;
            lfd = left_front_drive.getCurrentPosition();
            lbd = left_back_drive.getCurrentPosition();
            rfd = right_front_drive.getCurrentPosition();
            rbd = right_back_drive.getCurrentPosition();

            motorsX = (lfd + lbd + rfd + rbd) / 4;
            motorsY = (-lfd + lbd + rfd - rbd) / 4;
            motorsZ = (-lfd - lbd + rfd + rbd) / 4;
            double powerx = PIDX.update(motorsX);
            double powery = PIDY.update(motorsY);
            double powerz = PIDZ.update(motorsZ);

            double kx = 0.001;
            double ky = 0.001;
            double kz = 0.01;

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
}

