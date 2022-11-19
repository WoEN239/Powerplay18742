package org.firstinspires.ftc.teamcode.Robot;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private DcMotor motor1;
    private DcMotor motor2;
    private PidRegulator PIDZL1 = new PidRegulator(1.125/2, 0, 0);
    private PidRegulator PIDZL2 = new PidRegulator(1.125/2, 0, 0);
    double told;

    private LinearOpMode opMode;

    public Lift(HardwareMap hardwareMap, LinearOpMode _opMode) {
        opMode = _opMode;
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

   public void reset() {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void setPowers(double x) {
        motor1.setPower(x);
        motor2.setPower(x);
    }

    public enum LiftPosition {
        ZERO(-80), GROUND(700), LOW(2400), MIDDLE(4500), UP(4500);

        private LiftPosition(int value) {
            this.value = value;
        }

        public int value;
    }

    public void setPowersLimit(double x) {
        int pos1 = motor1.getCurrentPosition();
        int pos2 = motor2.getCurrentPosition();
        if (x > 0) {
            if (pos1 > LiftPosition.UP.value) {
                motor1.setPower(0);
            } else {
                motor1.setPower(x);
            }
            if (pos2 > LiftPosition.UP.value) {

                motor2.setPower(0);
            } else {
                motor2.setPower(x);
            }
        }
        else
        {
            if (pos1 < LiftPosition.ZERO.value) {
                motor1.setPower(0);
            } else {
                motor1.setPower(x);
            }
            if (pos2 < LiftPosition.ZERO.value) {
                motor2.setPower(0);
            } else {
                motor2.setPower(x);
            }
        }
    }

    public void displayEncoders() {
        opMode.telemetry.addData("lift1", motor1.getCurrentPosition());
        opMode.telemetry.addData("lift2", motor2.getCurrentPosition());
        opMode.telemetry.update();
    }

    public void setMotor(LiftPosition position) {
        double z = 0;
        double hight = position.value;

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

        while (((abs(err1)) > 100 && (abs(err2)) > 100) && tr < 4000 && opMode.opModeIsActive()) {
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

    }
}
