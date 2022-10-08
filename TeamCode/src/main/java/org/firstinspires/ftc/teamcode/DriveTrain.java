package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.*;
public class DriveTrain {
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    DcMotor leftRearMotor;
    DcMotor rightRearMotor;
    private PidRegulator PIDX=new PidRegulator(0.025,0.0000001,0.001);
    private PidRegulator PIDY=new PidRegulator(0.025,0.0000001,0.001);
    private PidRegulator PIDZ=new PidRegulator(0.025,0.0000001,0.001);
double crr=24 * 20 / (9.8 * PI);
    public DriveTrain(HardwareMap hardwareMap) {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftRearMotor = hardwareMap.dcMotor.get("leftRearMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        rightRearMotor = hardwareMap.dcMotor.get("rightRearMotor");
    }

    void reset() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void setPowers(double x, double y, double z) {
        double leftFrontMotorPower = x - y - z;
        double rightFrontMotorPower = x + y - z;
        double leftRearMotorPower = x + y - z;
        double rightRearMotorPower = x - y + z;
        leftFrontMotor.setPower(leftFrontMotorPower);
        rightFrontMotor.setPower(rightFrontMotorPower);
        leftRearMotor.setPower(leftRearMotorPower);
        rightRearMotor.setPower(rightRearMotorPower);
    }
    void setMotor(double x, double y, double z) {
        double lfd = leftFrontMotor.getCurrentPosition();
        double lbd = leftRearMotor.getCurrentPosition();
        double rfd = rightFrontMotor.getCurrentPosition();
        double rbd = rightRearMotor.getCurrentPosition();

        double motorsX = (lfd + lbd + rfd + rbd) / 4;
        double motorsY = (-lfd + lbd + rfd - rbd) / 4;
        double motorsZ = (-lfd - lbd + rfd + rbd) / 4;

        PIDX.target = x * crr;
        PIDY.target = y * crr;
        PIDZ.target = z * crr ;

        PIDX.update(motorsX);
        PIDY.update(motorsY);
        PIDZ.update(motorsZ);

        while ((abs( PIDX.err)) > 100 && (abs(PIDY.err)) > 100 && (abs(PIDZ.err)) > 100) {
            lfd = leftFrontMotor.getCurrentPosition();
            lbd = leftRearMotor.getCurrentPosition();
            rfd = rightFrontMotor.getCurrentPosition();
            rbd = rightRearMotor.getCurrentPosition();

            motorsX = (lfd + lbd + rfd + rbd) / 4;
            motorsY = (-lfd + lbd + rfd - rbd) / 4;
            motorsZ = (-lfd - lbd + rfd + rbd) / 4;
            double powerx = PIDX.update(motorsX);
            double powery =  PIDY.update(motorsY);
            double powerz = PIDZ.update(motorsZ);

            double kx = 0.001;
            double ky = 0.001;
            double kz = 0.01;

            leftFrontMotor.setPower(powerx - powery - powerz);
            rightFrontMotor.setPower(powerx + powery - powerz);
            leftRearMotor.setPower(powerx + powery - powerz);
            rightRearMotor.setPower(powerx - powery + powerz);

        }
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

