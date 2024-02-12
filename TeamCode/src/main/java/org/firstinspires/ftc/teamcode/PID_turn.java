package org.firstinspires.ftc.teamcode;
import android.transition.Slide;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
public class PID_turn {
    private double TargetAngle;
    private double errorSum = 0;
    private double Kp,Ki,Kd;

    public PID_turn(double target, double p, double i, double d){
        TargetAngle = target;

        Kp = p;
        Ki = i;
        Kd = d;
    }
    public double update(double currentAngle){

        double error = TargetAngle - currentAngle;

        error %= 360;
        error += 360;
        error %= 360;


        if (error > 180){
            error -= 360;
        }

        errorSum += error;

        if (Math.abs(error) < 2){
            errorSum = 0;
        }
        errorSum = Math.abs(errorSum) * Math.signum(error);

       double motorPower = 0.1 * Math.signum(error) + 0.9 * Math.tanh(Kp * error + Ki * errorSum);


        return motorPower;
    }
}
