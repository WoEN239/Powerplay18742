package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AiRRobot {
    public DriveTrain driveTrain;
    public Graber graber;
    public Lift lift;
    public Lightning lightning;
    public Odometry odometry;
    public LinearOpMode linearOpMode;

    public AiRRobot(LinearOpMode linearOpMode1) {
        linearOpMode = linearOpMode1;
        driveTrain = new DriveTrain(this);
        lift = new Lift(this);
        graber = new Graber(this);
        lightning = new Lightning(this);
        odometry = new Odometry(this);
    }
    public void allUpdate(){
        lift.update();
        lightning.update();
    }
}
