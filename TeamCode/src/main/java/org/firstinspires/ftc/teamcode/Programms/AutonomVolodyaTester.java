package org.firstinspires.ftc.teamcode.Programms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import static java.lang.Math.abs;

public class AutonomVolodyaTester extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor liftLeft = null;
    private DcMotor liftRight = null;

    Servo servo;
    Servo servo1;

    @Override

    public void runOpMode() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        liftLeft = hardwareMap.get(DcMotor.class, "motor1");
        liftRight = hardwareMap.get(DcMotor.class, "motor2");

        servo = hardwareMap.get(Servo.class, "Servo");
        servo1= hardwareMap.get(Servo.class, "Servo1");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        double motors1, motors2;

        while (true) {

            double lfd = leftFrontDrive.getCurrentPosition();
            double lbd = leftBackDrive.getCurrentPosition();
            double rfd = rightFrontDrive.getCurrentPosition();
            double rbd = rightBackDrive.getCurrentPosition();

            double lr = liftLeft.getCurrentPosition();
            double ll = liftRight.getCurrentPosition();

            double Servo = servo.getPosition();
            double Servo1 = servo1.getPosition();

            motors1 = (abs(lfd + lbd + rfd + rbd)) / 4;
            motors2 = (abs(lr + ll)) / 2;

            telemetry.addData("Левый передний мотор:", lfd);
            telemetry.addData("Левый задний мотор:", lbd);
            telemetry.addData("Правый передний мотор:", rfd);
            telemetry.addData("Правый задний мотор:", rbd);
            telemetry.addData("Сумма энкодоров с колёс:", motors1);
            telemetry.addData("Сумма энкодоров с лифта:", motors2);
            telemetry.addData("Позиция захвата:", Servo);
            telemetry.addData("Позиция дополнительного подъёмника:", Servo1);
            telemetry.update();

        }
    }
}