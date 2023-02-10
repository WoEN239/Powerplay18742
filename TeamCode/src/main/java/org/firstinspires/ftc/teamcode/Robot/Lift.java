package org.firstinspires.ftc.teamcode.Robot;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Config
public class Lift {
    public DcMotor motor1;
    public DcMotor motor2;
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
        ZERO(1), GROUND(100), LOW(780), MIDDLE(1180), UP(1230);

        LiftPosition(int value) {
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
        liftPosition=position;
        double hight = position.value;

        liftMode = LiftMode.AUTO;
        double l1 = motor1.getCurrentPosition();
        double l2 = motor2.getCurrentPosition();

        double motorsY = (l1 + l2) / 2;

        double target1 = hight;
        double target2 = hight;
        err1 = target1 - l1;
        err2 = target2 - l2;
        PIDZL1.update(motorsY);
        PIDZL2.update(motorsY);

        double t1 = (double) System.currentTimeMillis() / 1000.0;
        double t;
        double tr=0;

        while (!isAtPosition() && tr < 3 && aiRRobot.linearOpMode.opModeIsActive()) {
            t=(double) System.currentTimeMillis() / 1000.0;
            tr = t - t1;
            liftMode = LiftMode.AUTO;
            update();

        }
        motor1.setPower(0);
        motor2.setPower(0);

    }

    public static double POS_UP = 0.72;
    public static double POS_DOWN = 0.43;

    public void update() {
        if (aiRRobot.graber.getPosition())
            aiRRobot.graber.servo1.setPosition(POS_UP);
        else
            aiRRobot.graber.servo1.setPosition(POS_DOWN);

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
                motor1.setPower(Range.clip(poweryl1, -0.1, 0.6));
                motor2.setPower(Range.clip(poweryl2, -0.1, 0.6));
                break;
            case MANUALLIMIT:
                setPowersLimit(power);
                break;
            case MANUAl:
                setPowers(power);
                break;
        }
    }
    public boolean isAtPosition() {
        if(abs(err1)<5 && abs(err2)<5)
            return true;
        else
            return false;
    }
}
