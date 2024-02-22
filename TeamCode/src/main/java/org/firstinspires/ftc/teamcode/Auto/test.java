package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpMode;

public class test extends OpMode {
    @Override
    protected void run() {
        telemetryTfod();
        while (object == false){
            turnToGyroPower(90,0.1);
            initTfod();
        }
    }

    @Override
    protected void end() {

    }
}
