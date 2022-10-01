package org.firstinspires.ftc.teamcode;

public class PidRegulator {
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double ui = 0;
    double target = 0;


    public PidRegulator(double p, double i, double d) {
        kP=p;
        kI=i;
        kD=d;
    }


    public double update(double sensorValue) {


        double err=target-sensorValue;
        double up=err*kP;
        double ui+=(err*kI) ;
        double ud=err*kD;
    }
}
