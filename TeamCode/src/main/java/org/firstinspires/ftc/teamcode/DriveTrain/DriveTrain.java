package org.firstinspires.ftc.teamcode.DriveTrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.Side;
import org.openftc.easyopencv.OpenCvCamera;

public class DriveTrain {
    private DcMotorEx BR, BL, FR, FL;
    private BNO055IMU imu;
    private IMU Imu;
    private Telemetry telemetry;
    private LinearOpMode opMode;
    private OpenCvCamera camera;
    ElapsedTime runtime = new ElapsedTime();
    public static double Kp = 0.5, Ki = 0.2, Kd = 0.01;
    static final double WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference
    static final double COUNTS_PER_CM = 537.6 / WHEEL_DIAMETER_CM * Math.PI;//(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * PI);

    public DriveTrain(DcMotorEx BR, DcMotorEx BL, DcMotorEx FR, DcMotorEx FL, Telemetry telemetry, IMU imu) {
        this.BL = BL;
        this.BR = BR;
        this.FL = FL;
        this.FR = FR;

        this.Imu = imu;
        this.telemetry = telemetry;
    }

    public void setPID(double p, double i, double d) {
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
    }

    public void Roni(double y, double x, double rx, double botHeading){

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY - rotX - rx) / denominator;
        double backLeftPower = (rotY + rotX - rx) / denominator;
        double frontRightPower = (rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX + rx) / denominator;

        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);
        BR.setPower(frontRightPower);
        BR.setPower(backRightPower);

    }


    public void turnToGyro(double degrees){
        Imu.resetYaw();
        double error = degrees + Math.abs(Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        while (Math.abs(error) > 2) {
                double botAngle = Math.abs(Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

               // double ticks_per_cm = 537.6 / 9.6;
                double wantedPose = COUNTS_PER_CM * degrees;
                PID pid = new PID(0.5,0.2,0.1,0,0);
                pid.setWanted(wantedPose);
                FL.setPower(pid.update(FL.getCurrentPosition()));
                FR.setPower(pid.update(FR.getCurrentPosition()));

                BR.setPower(pid.update(BR.getCurrentPosition()));
                BL.setPower(pid.update(BL.getCurrentPosition()));
                error = degrees - botAngle;

            }
            FL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
        }



    public void driveTo(double cm){
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double positionWanted = 0;
        PID pid = new PID(0.5, 0.2, 0.1, 0, 0);

        positionWanted = cm * COUNTS_PER_CM;
        pid.setWanted((int)positionWanted);

        while (Math.abs(positionWanted)> Math.abs(FL.getCurrentPosition()) + 2){
            FL.setPower(pid.update(FL.getCurrentPosition()) / 2);
            BL.setPower(pid.update(FL.getCurrentPosition()) / 2);

            FR.setPower(pid.update(FL.getCurrentPosition()));
            BR.setPower(pid.update(FL.getCurrentPosition()));
        }

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void SideWalk(double cm){
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double positionWanted = 0;
        PID pid = new PID(0.5, 0.2, 0.1, 0, 0);

        positionWanted = cm * COUNTS_PER_CM;
        pid.setWanted((int)positionWanted);

        while (Math.abs(positionWanted) > Math.abs(FL.getCurrentPosition())){
            FL.setPower(-pid.update(FL.getCurrentPosition()));
            BL.setPower(pid.update(FL.getCurrentPosition()));

            FR.setPower(-pid.update(FL.getCurrentPosition()));
            BR.setPower(pid.update(FL.getCurrentPosition()));
        }
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}
