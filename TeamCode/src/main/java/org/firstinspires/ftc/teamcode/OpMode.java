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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


public abstract class OpMode extends LinearOpMode {

    protected TouchSensor touch;
    public DcMotorEx EncoderLeft,EncoderRight,EncoderCenter;
    protected DcMotorEx DriveFrontLeft, DriveFrontRight, DriveBackLeft, DriveBackRight, arm1, arm2, intake,intake2, ANGLE;
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
        DriveBackLeft.setDirection(DcMotor.Direction.REVERSE);
        DriveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DriveBackRight = hardwareMap.get(DcMotorEx.class, "BR");
        DriveBackRight.setDirection(DcMotor.Direction.FORWARD);
        DriveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1 = hardwareMap.get(DcMotorEx.class,"ELEVATOR R");
        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm1.setDirection(DcMotorEx.Direction.REVERSE);
        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        arm2 = hardwareMap.get(DcMotorEx.class,"ELEVATOR L");
        arm2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setDirection(DcMotorEx.Direction.FORWARD);
        arm2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotorEx.class,"WHEELS");
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake2 = hardwareMap.get(DcMotorEx.class,"WHEELS");
        intake2.setDirection(DcMotorEx.Direction.REVERSE);
        intake2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        ANGLE = hardwareMap.get(DcMotorEx.class,"ANGLE");
        ANGLE.setDirection(DcMotorEx.Direction.FORWARD);
        ANGLE.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

       /* EncoderLeft = hardwareMap.get(DcMotorEx.class,"LeftEncoder");
        EncoderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        EncoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        EncoderRight = hardwareMap.get(DcMotorEx.class,"RightEncoder");
        EncoderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        EncoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        EncoderCenter = hardwareMap.get(DcMotorEx.class,"CenterEncoder");
        EncoderCenter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        EncoderCenter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       */
        touch = hardwareMap.get(TouchSensor.class, "touch");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.gyroPowerMode = BNO055IMU.GyroPowerMode.FAST;
        parameters.gyroRange = BNO055IMU.GyroRange.DPS2000;
        imu.initialize(parameters);

        calibrateGyro();

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

    float getGyro() {
        return (imu.getAngularOrientation(EXTRINSIC, XYZ, AngleUnit.DEGREES).thirdAngle - gyroCalibration + 180) % 360 - 180;
    }

    void calibrateGyro() {
        gyroCalibration = (imu.getAngularOrientation().firstAngle + 180) % 360 - 180;
    }
    private float normalizeAngle(float angle) {
        return (angle + 540) % 360 - 180;
    }

    void move(double pow, double turn, boolean isDrift) {
        double frontLeftPower, frontRightPower, backRightPower, backLeftPower;
        if (isDrift) {
            backLeftPower = frontRightPower = Range.clip(pow + turn, -1.0, 1.0);
            backRightPower = frontLeftPower = Range.clip(pow - turn, -1.0, 1.0);
        } else {
            backLeftPower = frontLeftPower = Range.clip(pow + turn, -1.0, 1.0);
            backRightPower = frontRightPower = Range.clip(pow - turn, -1.0, 1.0);
        }

        DriveFrontRight.setPower(frontRightPower);
        DriveBackRight.setPower(backRightPower);
        DriveFrontLeft.setPower(frontLeftPower);
        DriveBackLeft.setPower(backLeftPower);
    }

    final protected void moveFreeWithGyro(double x, double y, double turn) {
        double pow, drift, virtualAngle, powToDriftRatio, realAngle, d = sqrt(x * x + y * y);
        double frontRightPower, frontLeftPower, backLeftPower, backRightPower;

        x = -x;

        if (x == 0) {
            realAngle = y > 0 ? 90 : -90;
        } else if (y == 0) {
            realAngle = x > 0 ? 0 : 180;
        } else {
            realAngle = toDegrees(atan(y / x));
            if (x < 0) {
                realAngle = 180 + realAngle;
            }
        }
        virtualAngle = realAngle - getGyro() - 180;

        virtualAngle = (virtualAngle + 360) % 360;

        powToDriftRatio = tan(toRadians(virtualAngle));

        drift = d / sqrt(powToDriftRatio * powToDriftRatio + 1);
        pow = drift * powToDriftRatio;

        // distinguish between both solutions to our equations
        if (virtualAngle > 90 && virtualAngle <= 270) {
            pow = -pow;
        } else {
            drift = -drift;
        }

        backLeftPower = frontRightPower = pow + drift;
        backRightPower = frontLeftPower = pow - drift;
        frontRightPower = Range.clip(frontRightPower - turn, -1, 1);
        backRightPower = Range.clip(backRightPower - turn, -1, 1);
        frontLeftPower = Range.clip(frontLeftPower + turn, -1, 1);
        backLeftPower = Range.clip(backLeftPower + turn, -1, 1);

        DriveFrontRight.setPower(frontRightPower);
        DriveBackRight.setPower(backRightPower);
        DriveFrontLeft.setPower(frontLeftPower);
        DriveBackLeft.setPower(backLeftPower);
    }
    final protected void lockedAngleMoveFreeWithGyro(double x, double y, float angle) {
        double pow, drift, virtualAngle, powToDriftRatio, realAngle, d = sqrt(x * x + y * y);
        double frontRightPower, frontLeftPower, backLeftPower, backRightPower;

        x = -x;

        if (x == 0) {
            realAngle = y > 0 ? 90 : -90;
        } else if (y == 0) {
            realAngle = x > 0 ? 0 : 180;
        } else {
            realAngle = toDegrees(atan(y / x));
            if (x < 0) {
                realAngle = 180 + realAngle;
            }
        }

        virtualAngle = realAngle - getGyro() - 180;

        virtualAngle = (virtualAngle + 360) % 360;

        powToDriftRatio = tan(toRadians(virtualAngle));

        drift = d / sqrt(powToDriftRatio * powToDriftRatio + 1);
        pow = drift * powToDriftRatio;

        // distinguish between both solutions to our equations
        if (virtualAngle > 90 && virtualAngle <= 270) {
            pow = -pow;
        } else {
            drift = -drift;
        }

        backLeftPower = frontRightPower = pow + drift;
        backRightPower = frontLeftPower = pow - drift;

        double s = normalizeAngle((getGyro() - angle)) / 100d;

        frontRightPower = Range.clip(frontRightPower - s, -1, 1);
        backRightPower = Range.clip(backRightPower - s, -1, 1);
        frontLeftPower = Range.clip(frontLeftPower + s, -1, 1);
        backLeftPower = Range.clip(backLeftPower + s, -1, 1);

        DriveFrontRight.setPower(frontRightPower);
        DriveBackRight.setPower(backRightPower);
        DriveFrontLeft.setPower(frontLeftPower);
        DriveBackLeft.setPower(backLeftPower);
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
