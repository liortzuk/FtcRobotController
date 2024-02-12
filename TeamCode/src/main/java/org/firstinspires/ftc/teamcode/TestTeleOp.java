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

@Config
@TeleOp
public class TestTeleOp extends OpMode {


    @Override
    protected void postInit() {
        Imu.resetYaw();
    }

    @Override
    public void run(){

         double positionWanted = 0;
         PID pid = new PID(0.01, 0, 0, 0, 0);

        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;//-1 to 1
            double turn = gamepad1.right_stick_x;
            double drift = gamepad1.left_stick_x;
            double botHeading = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botAngle = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            //Arava(forward, drift, turn);
            Roni(forward, drift, turn, botHeading);

            //lift power
            positionWanted += -gamepad2.left_stick_y * 100;

            if (positionWanted >= 2400){
                positionWanted = 2400;
            }else  if (positionWanted <= 0){
                positionWanted = 0;
            }

            pid.setWanted(positionWanted);
            armR.setPower(pid.update(armL.getCurrentPosition()));
            armL.setPower(pid.update(armL.getCurrentPosition()));



            if (gamepad1.options) {
                Imu.resetYaw();
            }


            if(gamepad2.left_bumper){
                RightServo.setPower(1);

            }
            else if(gamepad2.right_bumper){
                LeftServo.setPower(-1);
            }else {
                RightServo.setPower(0);
                LeftServo.setPower(0);
            }

            ///take in
            if(gamepad2.b){
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                trigger.setPosition(0);
                intake.setPower(1);

            }
            ///put out
            else if(gamepad2.a){
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intake.setPower(-1);

                angle.setPosition(0.625);
            }
            else {
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intake.setPower(0);
            }

            if(gamepad2.left_trigger != 0){
                AngleLift(0,-1);
                ANGLE.setPower(0);
            } else if (gamepad2.right_trigger != 0) {
                AngleLift(800,-1);
                ANGLE.setPower(0);
            }

/*
            if(gamepad2.left_trigger != 0 && ANGLE.getCurrentPosition() > -699){
                    ANGLE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    ANGLE.setPower(-gamepad2.left_trigger);
            } else if (gamepad2.right_trigger != 0 && ANGLE.getCurrentPosition() < 1100) {
                    ANGLE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    ANGLE.setPower(gamepad2.right_trigger);
            }
            else {
                ANGLE.setPower(0);
            }

 */

            if (gamepad2.x){
                RightServo.setPower(-1);
                ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamepad2.y){
                LeftServo.setPower(1);
            }

            if(gamepad2.dpad_up){
                armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Elevator(727);

                AngleLift(745,-1);
                ANGLE.setPower(0);

                Elevator(662);

                IntakePower(1400,1);


                armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            telemetry.addData("Left Lift: ", armL.getCurrentPosition());
            telemetry.addData("Right Lift: ", armR.getCurrentPosition());
            telemetry.addData("ANGLE: ", ANGLE.getCurrentPosition());
            telemetry.addData("FL power: ", DriveFrontLeft.getPower());
            telemetry.addData("BL power: ", DriveBackLeft.getPower());
            telemetry.addData("FR power: ", DriveFrontRight.getPower());
            telemetry.addData("BR power: ", DriveBackRight.getPower());
            telemetry.addData("Wheels: ", intake.getCurrentPosition());
            telemetry.addData("Gyro: ", botAngle);
            telemetry.addData("wanted: ", positionWanted);
            telemetry.addData("power: ", armL.getPower());
            telemetry.update();

        }

    }
    @Override
    protected void end() {

    }
}
