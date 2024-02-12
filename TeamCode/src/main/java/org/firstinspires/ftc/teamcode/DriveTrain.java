package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static java.lang.Math.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config
public class DriveTrain {
    private DcMotorEx BR, BL, FR, FL;
    private BNO055IMU imu;
    private IMU Imu;
    private Telemetry telemetry;
    private LinearOpMode opMode;
    private OpenCvCamera camera;
    ElapsedTime runtime = new ElapsedTime();
    private Side side = Side.BLUE;

    private int cameraMonitorViewId = 0;
    private boolean align = false, streaming;

    public static double Kp = 0.5, Ki = 0.2, Kd = 0.01; // Ki = .1 / finalError;
    public static double iLimit = 0, maxSpeed = .4, maxAngularSpeed = 1;
    public int tics = 0;

    private double BOT_WIDTH = 23;
    private double cAngle = 0; // current angle
    private double gyroCalibration = 0;
    public boolean isBusy = false;

    double dt = 0, lastRuntime = 0, sumError = 0, errorRate = 0, lastError = 0, outputSpeed = 0;
    ElapsedTime pidRuntime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: Rev DcMotor Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 18.9;     // No External Gearing --> 1
    static final double WHEEL_DIAMETER_CM = 9;     // For figuring circumference
    static final double COUNTS_PER_CM = 537.6 / 9.6;//(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * PI);

    public static double time = 2;

    public DriveTrain(DcMotorEx BR, DcMotorEx BL, DcMotorEx FR, DcMotorEx FL, Telemetry telemetry, IMU imu, LinearOpMode opMode) {
        this.BL = BL;
        this.BR = BR;
        this.FL = FL;
        this.FR = FR;
        this.opMode = opMode;

        this.Imu = imu;
        this.telemetry = telemetry;
    }

    public void setPID(double p, double i, double d) {
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
    }

    public void moveTo(int cm) {

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double ticks_per_cm = 537.6 / 9.6;
        int setPoint = cm * (int) ticks_per_cm;

        double kp = 0.5;
        double ki = 0.2;
        double kd = 0.01;

        double errorSum = 0;
        double lastTimeSamp = 0;
        double lastError = 0;

        lastTimeSamp = runtime.seconds();

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double error = cm;

        while (abs(error) > 1 && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            double dt = runtime.seconds() - lastTimeSamp;

            double MotorPositionFL = BL.getCurrentPosition() / ticks_per_cm;

            error = cm - Math.abs(MotorPositionFL);

            if (Math.abs(error) < setPoint){
                errorSum += dt * error;
            }

            double errorRate = (error - lastError) / dt;

            double outputSpeed = kp * error + ki * errorSum + kd * errorRate;

            FL.setPower(-outputSpeed / 4);
            BL.setPower(outputSpeed / 4);
            FR.setPower(outputSpeed);
            BR.setPower(outputSpeed);

            lastTimeSamp = runtime.seconds();
            lastError = error;


            telemetry.addData("FL position", FL.getCurrentPosition());
            telemetry.addData("FR position", BL.getCurrentPosition());
            telemetry.addData("BL position", FR.getCurrentPosition());
            telemetry.addData("BR position", BR.getCurrentPosition());
            telemetry.addData("Error: ", error);
            telemetry.update();
        }

        BR.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
