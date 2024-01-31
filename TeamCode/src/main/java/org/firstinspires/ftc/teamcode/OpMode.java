package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YXZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.asin;
import static java.lang.Math.atan;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;
import static java.lang.StrictMath.toDegrees;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.OpenCV.Belman_camera;
import org.openftc.apriltag.AprilTagDetection;
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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID;

import java.util.ArrayList;


public abstract class OpMode extends LinearOpMode {

    protected TouchSensor touch;
    protected CRServo LeftServo, RightServo;
    protected DcMotorEx DriveFrontLeft, DriveFrontRight, DriveBackLeft, DriveBackRight, armR, armL, intake, ANGLE;
    protected ElapsedTime runtime = new ElapsedTime();
    protected float gyroCalibration = 0;
    protected BNO055IMU imu;

    public DriveTrain drivetrain;
    FtcDashboard dashboard;

    void initialize() {
        Side side;

        DriveFrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        DriveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        DriveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveFrontRight = hardwareMap.get(DcMotorEx.class, "FR");
        DriveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        DriveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveBackLeft = hardwareMap.get(DcMotorEx.class, "BL");
        DriveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        DriveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveBackRight = hardwareMap.get(DcMotorEx.class, "BR");
        DriveBackRight.setDirection(DcMotor.Direction.REVERSE);
        DriveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armR = hardwareMap.get(DcMotorEx.class,"ELEVATOR R");
        armR.setDirection(DcMotorEx.Direction.FORWARD);
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        armL = hardwareMap.get(DcMotorEx.class,"ELEVATOR L");
        armL.setDirection(DcMotorEx.Direction.FORWARD);
        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotorEx.class,"WHEELS");
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        ANGLE = hardwareMap.get(DcMotorEx.class,"ANGLE");
        ANGLE.setDirection(DcMotorEx.Direction.FORWARD);
        ANGLE.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        LeftServo = hardwareMap.get(CRServo.class, "Left Servo");
        RightServo = hardwareMap.get(CRServo.class, "Right Servo");

      //  touch = hardwareMap.get(TouchSensor.class, "touch");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "imu";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        telemetry.addLine("Waiting for start");
        telemetry.update();

       // Find();

    }


    @Override
    public void runOpMode() throws InterruptedException  {
        initialize();
        waitForStart();
        postInit();

        dashboard = FtcDashboard.getInstance();
        drivetrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, imu, this);

        if (opModeIsActive()) {
            run();
        }

        end();
    }

    protected void postInit() {

    }
    protected abstract void run();

    protected abstract void end();

    public void Roni(double y, double x, double rx, double botHeading){

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

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

    public void Elevator(int Right_Target, int Left_Target,double PowerR, double PowerL){

        armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armR.setTargetPosition(Right_Target);
        armL.setTargetPosition(Left_Target);

        armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (opModeIsActive()  && armL.isBusy() && armR.isBusy()){
            armR.setPower(PowerR);
            armL.setPower(PowerL);
        }
        armR.setPower(0);
        armL.setPower(0);


    }

    public void AngleLift(int position, double power){
        ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ANGLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ANGLE.setTargetPosition(position);
        ANGLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ANGLE.setPower(power);


        while (opModeIsActive() && ANGLE.isBusy()){
            idle();
        }


    }
    public void IntakePower(int position, double power){
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setTargetPosition(position);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake.setPower(power);

        while (opModeIsActive()  && intake.isBusy()){
            LeftServo.setPower(-.15);
            RightServo.setPower(.15);
        }

    }
    public void resetAngle(){
        gyroCalibration = (imu.getAngularOrientation().firstAngle + 180) % 360 - 180;
    }
}
