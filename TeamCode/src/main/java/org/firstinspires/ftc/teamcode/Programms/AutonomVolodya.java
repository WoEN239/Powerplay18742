package org.firstinspires.ftc.teamcode.Programms;
//импортируем библиотеки----------------------------------------------------------------------------
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import static java.lang.Math.abs;
//класс AutonomVolodya------------------------------------------------------------------------------
public class AutonomVolodya extends LinearOpMode {
    //назначаем приватность моторам-----------------------------------------------------------------
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    //----------------------------------------------------------------------------------------------
    @Override
    //инициализирующияся программа при запуске(это база)--------------------------------------------
    public void runOpMode() {
        //
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        StopMotors();

        waitForStart();
    }

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

    void Turn(int target, double power) {

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
            telemetry.addData("Функция:", "Turn");
            telemetry.addData("Цель:", target);
            telemetry.update();
        }
        StopMotors();
    }

    void LeftRight(int target, double power) {
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
            telemetry.addData("Функция:", "LeftRight");
            telemetry.update();
        }
        StopMotors();
    }

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
}