package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

public class PidRegulator {
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double ui = 0;
    double target = 0;
    double errold;
    double told;
    double err;

    public PidRegulator(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }


    public double update(double sensorValue) {

        double time = System.currentTimeMillis() / 1000.0;
        err = target - sensorValue;
        double up = err * kP;
        ui += (err * kI) * (time - told);
        if (abs(ui) > 1) {
            ui = 1;
        }
        double ud = (err - errold) * kD / (time - told);
        errold = err;
        told = time;
        return up + ud + ui;

    }
}