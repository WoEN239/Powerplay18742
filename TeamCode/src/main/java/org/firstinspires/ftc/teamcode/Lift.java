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
    private PidRegulator PIDZL1 = new PidRegulator(10.125, 0.0000001, 0.001);
    private PidRegulator PIDZL2 = new PidRegulator(10.125, 0.0000001, 0.001);
    double crr = 24 * 20 / (2.5 * PI);
    double told;

    private LinearOpMode opMode;

    public Lift(HardwareMap hardwareMap, LinearOpMode _opMode) {
        opMode = _opMode;
        motor1 = hardwareMap.dcMotor.get("Lift1");
        motor2 = hardwareMap.dcMotor.get("Lift2");
        svet = hardwareMap.dcMotor.get("color1");
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    void reset() {
        opMode = _opMode;
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    void setMotor(double position) {
        double z = 0;
        double hight=0;
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

        PIDZL1.target=hight;
        PIDZL2.target=hight;
        PIDZL1.update(motorsY);
        PIDZL2.update(motorsY);

        double t1 = System.currentTimeMillis() / 1000.0;
        double t = 0;
        double tr = t - told;
        svet.setPower(1);
        while (((abs(PIDZL1.err)) > 100 || (abs(PIDZL2.err)) > 100) && tr < 5000 && opMode.opModeIsActive()) {
            t = System.currentTimeMillis() / 1000.0;
            tr = t - t1;
            l1 = motor1.getCurrentPosition();
            l2 = motor2.getCurrentPosition();

            motorsY = (l1 + l2) / 2;
            double poweryl1 = PIDZL1.update(motorsY);
            double poweryl2 = PIDZL1.update(motorsY);

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
