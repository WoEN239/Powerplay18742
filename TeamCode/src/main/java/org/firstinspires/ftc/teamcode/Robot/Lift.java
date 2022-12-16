package org.firstinspires.ftc.teamcode.Robot;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private DcMotor motor1;
    private DcMotor motor2;
    private PidRegulator PIDZL1 = new PidRegulator(0.8125 / 251.5, 0, 0);
    private PidRegulator PIDZL2 = new PidRegulator(0.8125 / 251.5, 0, 0);
    double told;
   double err1=0;
   double err2=0;

    public double power = 0;
    public LiftPosition liftPosition = LiftPosition.ZERO;

    public enum LiftMode {
        AUTO, MANUAl, MANUALLIMIT;
    }

    public LiftMode liftMode = LiftMode.AUTO;
    AiRRobot aiRRobot;
    public Lift(AiRRobot robot) {
        aiRRobot=robot;
        motor1 = aiRRobot.linearOpMode.hardwareMap.dcMotor.get("motor1");
        motor2 = aiRRobot.linearOpMode.hardwareMap.dcMotor.get("motor2");
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public double encoders() {
        double m1 = motor1.getCurrentPosition();
        double m2 = motor2.getCurrentPosition();
        double m0 = (m1 + m2) / 2;
        return m0;
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
        aiRRobot.linearOpMode.telemetry.addData("lift1", motor1.getCurrentPosition());
        aiRRobot.linearOpMode.telemetry.addData("lift2", motor2.getCurrentPosition());
        aiRRobot.linearOpMode.telemetry.update();
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

        double t1 = (double) System.currentTimeMillis() / 1000.0;
        double t;
        double tr=0;

        while (!isAtPosiotoin() && tr < 5 && aiRRobot.linearOpMode.opModeIsActive()) {
            t=(double) System.currentTimeMillis() / 1000.0;
            tr=t-t1;
            liftMode=LiftMode.AUTO;
            update();

        }
        motor1.setPower(0);
        motor2.setPower(0);

    }

    public void update() {
        switch (liftMode) {
            case AUTO:
                double target1 = liftPosition.value;
                double target2 = liftPosition.value;
                double l1 = motor1.getCurrentPosition();
                double l2 = motor2.getCurrentPosition();
                err1 = target1 - l1;
                err2 = target2 - l2;
                double poweryl1 = PIDZL1.update(err1);
                double poweryl2 = PIDZL2.update(err2);
                motor1.setPower(poweryl1);
                motor2.setPower(poweryl2);
                break;
            case MANUALLIMIT:
                setPowersLimit(power);
                break;
            case MANUAl:
                setPowers(power);
                break;
        }
    }
    public boolean isAtPosiotoin() {
        if(abs(err1)<5 && abs(err2)<5)
            return true;
        else
            return false;
    }
}
