package org.firstinspires.ftc.teamcode.Robot;

import static java.lang.Math.abs;

public class PidRegulator {
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double ui = 0;
    double errold;
    double told;

    public PidRegulator(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }


    public double update(double err) {

        double time = System.currentTimeMillis() / 1000.0;
        double up = err * kP;
        ui += (err * kI) * (time - told);
        if (abs(ui) > 0.25) {
            ui = 0.25;
        }
        double ud = (err - errold) * kD / (time - told);
        errold = err;
        told = time;
        return up + ud + ui;
    }
    public void reset(){
        ui=0;
        told=System.currentTimeMillis()/1000.0;
    }
}
