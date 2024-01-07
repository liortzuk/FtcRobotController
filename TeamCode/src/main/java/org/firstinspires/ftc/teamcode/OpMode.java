package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static java.lang.Math.PI;
import static java.lang.Math.asin;
import static java.lang.Math.atan;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;
import static java.lang.StrictMath.toDegrees;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public abstract class OpMode extends LinearOpMode {

    protected TouchSensor touch;
    protected CRServo LeftServo, RightServo;
    public DcMotorEx EncoderLeft,EncoderRight,EncoderCenter;
    protected DcMotorEx DriveFrontLeft, DriveFrontRight, DriveBackLeft, DriveBackRight, armR, armL, intake, ANGLE;
    protected ElapsedTime runtime = new ElapsedTime();
    protected float gyroCalibration = 0;
    protected BNO055IMU imu;

    void initialize() {

        DriveFrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        DriveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        DriveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DriveFrontRight = hardwareMap.get(DcMotorEx.class, "FR");
        DriveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        DriveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DriveBackLeft = hardwareMap.get(DcMotorEx.class, "BL");
        DriveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        DriveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DriveBackRight = hardwareMap.get(DcMotorEx.class, "BR");
        DriveBackRight.setDirection(DcMotor.Direction.REVERSE);
        DriveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armR = hardwareMap.get(DcMotorEx.class,"ELEVATOR R");
        armR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armR.setDirection(DcMotorEx.Direction.FORWARD);
        armR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        armL = hardwareMap.get(DcMotorEx.class,"ELEVATOR L");
        armL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armL.setDirection(DcMotorEx.Direction.FORWARD);
        armL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotorEx.class,"WHEELS");
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        ANGLE = hardwareMap.get(DcMotorEx.class,"ANGLE");
        ANGLE.setDirection(DcMotorEx.Direction.FORWARD);
        ANGLE.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        LeftServo = hardwareMap.get(CRServo.class, "Left Servo");
        RightServo = hardwareMap.get(CRServo.class, "Right Servo");

        touch = hardwareMap.get(TouchSensor.class, "touch");

        telemetry.addLine("Waiting for start");
        telemetry.update();


    }


    @Override
    public void runOpMode() throws InterruptedException  {
        initialize();
        waitForStart();
        postInit();
        if (opModeIsActive()) {
            run();
        }

        end();
    }

    protected void postInit() {

    }
    protected abstract void run();

    protected abstract void end();

    public void power(double speed){
        DriveFrontLeft.setPower(speed);
        DriveFrontRight.setPower(-speed);
        DriveBackLeft.setPower(speed);
        DriveBackRight.setPower(-speed);
    }

    public void Roni(double y, double x, double rx, double botHeading){

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        DriveFrontLeft.setPower(frontLeftPower);
        DriveBackLeft.setPower(backLeftPower);
        DriveFrontRight.setPower(frontRightPower);
        DriveBackRight.setPower(backRightPower);
    }

/*
    public void odomatry(){
        //distance between n1 & n2
        double L =0;
        // distance between n1, n2 & n3
        double B =0;
        //wheel radius
        double R =0;
        //ticks per rev
        double N =0;
        //cm per ticks
        double cm_per_ticks = 2.0*Math.PI*R/N;

        double x =0;
        double y = 0;
        double x1 = 0;
        double y1 = 0;
        // x distance traveled
        double s1 = cm_per_ticks*(EncoderLeft.getCurrentPosition()+EncoderRight.getCurrentPosition())/2;
        //feida
        double s2 = cm_per_ticks*(EncoderRight.getCurrentPosition()- EncoderLeft.getCurrentPosition())/2;
        // y distance traveled
        double s3 = cm_per_ticks*(EncoderCenter.getCurrentPosition()-B*EncoderRight.getCurrentPosition()-EncoderLeft.getCurrentPosition()/L);

        //field x
        x1 = x +s1*Math.cos(s2)-s3*Math.sin(s2);
        // field y
        y1 = y + s1*Math.sin(s2)- s3*Math.cos(s2);
    }
 */
}
