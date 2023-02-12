//##################################################################################################
//    v             o             l              o              d              y              a
//##################################################################################################

/*В данный момент нужно протестить как работают все функции по данным критериям:
○ точность показателей энкодеров
○ точность траектории
○ время
○ функциональность
○ роботоспособность
А вот что нужно сделать в коде:
○ проверить и настроить значения захвата
○ проверить и настроить значения дополнительного подъёмника
○ добавить павороты по градусам*/

//-----------------------------------------CODE-----------------------------------------------------
package org.firstinspires.ftc.teamcode.Programms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import static java.lang.Math.abs;

public class AutonomVolodya extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor liftLeft = null;
    private DcMotor liftRight = null;

    Servo servo;
    Servo servo1;

    @Override

    //главная программа-----------------------------------------------------------------------------
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

        StopMotors();

        waitForStart();
        //функции-----------------------------------------------------------------------------------

    }

    //стоп моторы, сброс энкодера и использование энкодеров-----------------------------------------
    void StopMotors(){

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //паворот налево--------------------------------------------------------------------------------
    void TurnLeft(int target, double power) {

        double motors = 0;

        while (motors > target) {

            if(motors > target * 0.65){

                power = power * 0.5;

            }

            double lfd = leftFrontDrive.getCurrentPosition();
            double lbd = leftBackDrive.getCurrentPosition();
            double rfd = rightFrontDrive.getCurrentPosition();
            double rbd = rightBackDrive.getCurrentPosition();

            motors = (abs(lfd + lbd + rfd + rbd)) / 4;

            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);

            telemetry.addData("Левый передний мотор:", lfd);
            telemetry.addData("Левый задний мотор:", lbd);
            telemetry.addData("Правый передний мотор:", rfd);
            telemetry.addData("Правый задний мотор:", rbd);
            telemetry.addData("Мощность двигателя:", power);
            telemetry.addData("Сумма энкодоров с моторов:", motors);
            telemetry.addData("Функция:", "TurnLeft");
            telemetry.addData("Цель:", target);
            telemetry.update();

        }

        StopMotors();

    }

    //паворот впрово--------------------------------------------------------------------------------
    void TurnRight(int target, double power) {

        double motors = 0;

        while (motors > target) {

            if(motors > target * 0.65){

                power = power * 0.5;

            }

            double lfd = leftFrontDrive.getCurrentPosition();
            double lbd = leftBackDrive.getCurrentPosition();
            double rfd = rightFrontDrive.getCurrentPosition();
            double rbd = rightBackDrive.getCurrentPosition();

            motors = (abs(lfd + lbd + rfd + rbd)) / 4;

            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);

            telemetry.addData("Левый передний мотор:", lfd);
            telemetry.addData("Левый задний мотор:", lbd);
            telemetry.addData("Правый передний мотор:", rfd);
            telemetry.addData("Правый задний мотор:", rbd);
            telemetry.addData("Мощность двигателя:", power);
            telemetry.addData("Сумма энкодоров с моторов:", motors);
            telemetry.addData("Функция:", "TurnRight");
            telemetry.addData("Цель:", target);
            telemetry.update();

        }

        StopMotors();

    }

    //движение влево--------------------------------------------------------------------------------
    void Left(int target, double power) {

        double motors = 0;

        while (motors > target) {

            if(motors > target * 0.65){

                power = power * 0.5;

            }

            double lfd = leftFrontDrive.getCurrentPosition();
            double lbd = leftBackDrive.getCurrentPosition();
            double rfd = rightFrontDrive.getCurrentPosition();
            double rbd = rightBackDrive.getCurrentPosition();

            motors = (abs(lfd + lbd + rfd + rbd)) / 4;

            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);

            telemetry.addData("Левый передний мотор:", lfd);
            telemetry.addData("Левый задний мотор:", lbd);
            telemetry.addData("Правый передний мотор:", rfd);
            telemetry.addData("Правый задний мотор:", rbd);
            telemetry.addData("Мощность двигателя:", power);
            telemetry.addData("Сумма энкодоров с моторов:", motors);
            telemetry.addData("Цель:", target);
            telemetry.addData("Функция:", "Left");
            telemetry.update();

        }

        StopMotors();

    }

    //движение влево--------------------------------------------------------------------------------
    void Right(int target, double power) {

        double motors = 0;

        while (motors > target) {

            if(motors > target * 0.65){

                power = power * 0.5;

            }

            double lfd = leftFrontDrive.getCurrentPosition();
            double lbd = leftBackDrive.getCurrentPosition();
            double rfd = rightFrontDrive.getCurrentPosition();
            double rbd = rightBackDrive.getCurrentPosition();

            motors = (abs(lfd + lbd + rfd + rbd)) / 4;

            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);

            telemetry.addData("Левый передний мотор:", lfd);
            telemetry.addData("Левый задний мотор:", lbd);
            telemetry.addData("Правый передний мотор:", rfd);
            telemetry.addData("Правый задний мотор:", rbd);
            telemetry.addData("Мощность двигателя:", power);
            telemetry.addData("Сумма энкодоров с моторов:", motors);
            telemetry.addData("Цель:", target);
            telemetry.addData("Функция:", "Left");
            telemetry.update();

        }

        StopMotors();

    }

    //движение вперёд-------------------------------------------------------------------------------
    void Forward(int target, double power) {

        double motors = 0;

        while (motors > target) {

            if(motors > target * 0.65){

                power = power * 0.5;

            }

            double lfd = leftFrontDrive.getCurrentPosition();
            double lbd = leftBackDrive.getCurrentPosition();
            double rfd = rightFrontDrive.getCurrentPosition();
            double rbd = rightBackDrive.getCurrentPosition();

            motors = (abs(lfd + lbd + rfd + rbd)) / 4;

            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);

            telemetry.addData("Левый передний мотор:", lfd);
            telemetry.addData("Левый задний мотор:", lbd);
            telemetry.addData("Правый передний мотор:", rfd);
            telemetry.addData("Правый задний мотор:", rbd);
            telemetry.addData("Мощность двигателя:", power);
            telemetry.addData("Сумма энкодоров с моторов:", motors);
            telemetry.addData("Цель:", target);
            telemetry.addData("Функция:", "Forward");
            telemetry.update();

        }

        StopMotors();

    }

    //движение назад--------------------------------------------------------------------------------
    void Back(int target, double power) {

        double motors = 0;

        while (motors > target) {

            if(motors > target * 0.65){

                power = power * 0.5;

            }

            double lfd = leftFrontDrive.getCurrentPosition();
            double lbd = leftBackDrive.getCurrentPosition();
            double rfd = rightFrontDrive.getCurrentPosition();
            double rbd = rightBackDrive.getCurrentPosition();

            motors = (abs(lfd + lbd + rfd + rbd)) / 4;

            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(-power);

            telemetry.addData("Левый передний мотор:", lfd);
            telemetry.addData("Левый задний мотор:", lbd);
            telemetry.addData("Правый передний мотор:", rfd);
            telemetry.addData("Правый задний мотор:", rbd);
            telemetry.addData("Мощность двигателя:", power);
            telemetry.addData("Сумма энкодоров с моторов:", motors);
            telemetry.addData("Цель:", target);
            telemetry.addData("Функция:", "Back");
            telemetry.update();

        }

        StopMotors();

    }

    //лифт вверх------------------------------------------------------------------------------------
    void LiftUp(int target, double power) {

        double motors = 0;

        while (motors > target) {

            if(motors > target * 0.65){

                power = power * 0.5;

            }

            double lr = liftRight.getCurrentPosition();
            double ll = liftLeft.getCurrentPosition();

            motors = (abs(lr + ll)) / 2;

            liftRight.setPower(power);
            liftLeft.setPower(power);

            telemetry.addData("Левый мотор от лифта:", liftLeft);
            telemetry.addData("Правый мотор от лифта:", liftRight);
            telemetry.addData("Мощность двигателя:", power);
            telemetry.addData("Сумма энкодоров с моторов:", motors);
            telemetry.addData("Цель:", target);
            telemetry.addData("Функция:", "LiftUp");
            telemetry.update();

        }

        StopMotors();

    }

    //лифт вниз-------------------------------------------------------------------------------------
    void LiftDown(int target, double power) {

        double motors = 0;

        while (motors > target) {

            if(motors > target * 0.65){

                power = power * 0.5;

            }

            double lr = liftRight.getCurrentPosition();
            double ll = liftLeft.getCurrentPosition();

            motors = (abs(lr + ll)) / 2;

            liftRight.setPower(-power);
            liftLeft.setPower(-power);

            telemetry.addData("Левый мотор от лифта:", liftLeft);
            telemetry.addData("Правый мотор от лифта:", liftRight);
            telemetry.addData("Мощность двигателя:", power);
            telemetry.addData("Сумма энкодоров с моторов:", motors);
            telemetry.addData("Цель:", target);
            telemetry.addData("Функция:", "LiftDown");
            telemetry.update();

        }

        StopMotors();

    }

    //закрыть захват---------------------------------------------------------------------------------
    void CloseGraber(){
        servo.setPosition(0.75);
    }

    //открытьзахват---------------------------------------------------------------------------------
    void OpenGraber(){
        servo.setPosition(1.00);
    }

    //Поднять дополнительный подъёмник--------------------------------------------------------------
    void MiniLiftUP(){
        servo.setPosition(0.75);
    }

    //пустить дополнительный подъёмник--------------------------------------------------------------
    void MiniLiftDown(){
        servo.setPosition(1.00);
    }

}