package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor svet;
    private PidRegulator PIDZL1 = new PidRegulator(1.125, 0, 0);
    private PidRegulator PIDZL2 = new PidRegulator(1125, 0, 0);
    double crr = 24 * 20 / (2.5 * PI);
    double told;

    private LinearOpMode opMode;

    public Lift(HardwareMap hardwareMap, LinearOpMode _opMode) {
        opMode = _opMode;
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        svet = hardwareMap.dcMotor.get("svet");
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    void reset() {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    void setMotor(double position) {
        double z = 0;
        double hight = 0;
        if (position == 0) {
            hight = 0 * crr;
        }
        if (position == 1) {
            hight = 3 * crr;
        }
        if (position == 2) {
            hight = 37 * crr;
        }
        if (position == 3) {
            hight = 62 * crr;
        }
        if (position == 4) {
            hight = 87 * crr;
        }
        double l1 = motor1.getCurrentPosition();
        double l2 = motor2.getCurrentPosition();

        double motorsY = (l1 + l2) / 2;

        double target1 = hight;
        double target2 = hight;
        double err1 = target1 - l1;
        double err2 = target2 - l2;
        PIDZL1.update(motorsY);
        PIDZL2.update(motorsY);

        double t1 = System.currentTimeMillis() / 1000.0;
        double t = 0;
        double tr = t - told;
        svet.setPower(1);
        while (((abs(err1)) > 100 || (abs(err2)) > 100) && tr < 1500 && opMode.opModeIsActive()) {
            t = System.currentTimeMillis() / 1000.0;
            tr = t - t1;
            l1 = motor1.getCurrentPosition();
            l2 = motor2.getCurrentPosition();

            target1 = hight;
            target2 = hight;
            err1 = target1 - l1;
            err2 = target2 - l2;
            double poweryl1 = PIDZL1.update(err1);
            double poweryl2 = PIDZL1.update(err2);

            motor1.setPower(poweryl1);
            motor2.setPower(poweryl2);
            told = t;

        }
        motor1.setPower(0);
        motor2.setPower(0);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        svet.setPower(0);
    }
}
