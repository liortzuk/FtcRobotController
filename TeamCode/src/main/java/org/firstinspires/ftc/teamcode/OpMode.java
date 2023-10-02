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

    protected DcMotorEx DriveFrontLeft, DriveFrontRight, DriveBackLeft, DriveBackRight;
    protected ElapsedTime runtime = new ElapsedTime();
    protected float gyroCalibration = 0;
    protected BNO055IMU imu;

    void initialize() {

        DriveFrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        DriveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        DriveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DriveFrontRight = hardwareMap.get(DcMotorEx.class, "FR");
        DriveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        DriveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DriveBackLeft = hardwareMap.get(DcMotorEx.class, "BL");
        DriveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        DriveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DriveBackRight = hardwareMap.get(DcMotorEx.class, "BR");
        DriveBackRight.setDirection(DcMotor.Direction.FORWARD);
        DriveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
    public void runOpMode() {
        initialize();
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
}
