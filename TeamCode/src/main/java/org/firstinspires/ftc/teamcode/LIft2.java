package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LIft2 {
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor svet;
    private LinearOpMode opMode;
    double crr = 24 * 20 / (9.8 * PI);
    double told;

    public Lift2(HardwareMap hardwareMap, LinearOpMode _opMode) {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        svet = hardwareMap.get(DcMotor.class, "svet");
        opMode = _opMode;
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        svet.setDirection(DcMotor.Direction.FORWARD);
    }

    void reset2() {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void LiftPosition(double position) {
        double hight = 0;
        if (position == 0) {
            hight = 0 * crr;
        }
        if (position == 1) {
            hight = 25 * crr;
        }
        if (position == 2) {
            hight = 50 * crr;
        }
        if (position == 3) {
            hight = 90 * crr;
        }
        double l1 = motor1.getCurrentPosition();
        double l2 = motor2.getCurrentPosition();
        double motorsY = (l1 + l2) / 2;
        double t1 = System.currentTimeMillis() / 1000.0;
        double t = 0;
        double tr = t - told;

        while (((abs(.err)) > 100 || (abs(.err)) >100) && tr< 5000 && opMode.opModeIsActive()){
            t = System.currentTimeMillis() / 1000.0;
            tr = t - t1;
            l1 = motor1.getCurrentPosition();
            l2 = motor2.getCurrentPosition();
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }
}


