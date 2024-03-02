package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;import static java.lang.Math.abs;
import org.firstinspires.ftc.teamcode.DriveTrain.DriveTrain;

import org.firstinspires.ftc.teamcode.Elevator.Elevator;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {


    @Override
    protected void postInit() {
        Imu.resetYaw();
    }

    @Override
    public void run(){
         Elevator elevator = new Elevator(armL, armR, intake, ANGLE, LeftServo, RightServo, trigger, angle, telemetry);
         DriveTrain driveTrain = new DriveTrain(DriveBackLeft, DriveBackRight, DriveFrontRight, DriveFrontLeft, telemetry, Imu);
         double positionWanted = 0;
         PID pid = new PID(0.01, 0, 0, 0, 0);

        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;//-1 to 1
            double turn = gamepad1.right_stick_x;
            double drift = gamepad1.left_stick_x;
            double botHeading = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botAngle = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            driveTrain.Roni(forward, drift, turn, botHeading);

            if (Math.abs(-gamepad2.left_stick_y) > 0.2){
                armR.setPower(-gamepad2.left_stick_y);
                armL.setPower(-gamepad2.left_stick_y);
            }else {
                armR.setPower(0);
                armL.setPower(0);
            }

            if (gamepad1.options) {
                Imu.resetYaw();
            }

            if (gamepad1.x){
                angle.setPosition(0.625);
                trigger.setPosition(0);

            }

            if (gamepad1.b){
                angle.setPosition(0.2);
                angle.setPosition(0.5);
            }



            ///take in
            if(gamepad2.b){
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intake.setPower(1);

            }
            ///put out
            else if(gamepad2.a){
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intake.setPower(-1);
            }
            else {
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intake.setPower(0);
            }

            if(gamepad2.left_bumper){
                elevator.AngleLift(0,1);
            }
            else if(gamepad2.right_bumper){
                elevator.AngleLift(800,-1);
            }

            if(gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0) {
                RightServo.setPower(gamepad2.left_trigger);
                LeftServo.setPower(-gamepad2.right_trigger);
            }else {
                RightServo.setPower(0);
                LeftServo.setPower(0);
            }

            if (gamepad2.x){
                RightServo.setPower(-1);
            }
            if(gamepad2.y){
                LeftServo.setPower(1);
            }

            if(gamepad2.dpad_up){
                ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                elevator.Elevator(707);

                elevator.AngleLift(665,-1);
                ANGLE.setPower(0);

                elevator.Elevator(680);

                elevator.IntakePower(700,1);

                elevator.Elevator(1300);

                elevator.AngleLift(800,-1);

            }

            if (gamepad2.dpad_down){
                elevator.AngleLift(0,1);
                elevator.Elevator(10);
            }

            if(gamepad2.dpad_right){
                ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevator.AngleLift(738,1);
            }

            telemetry.addData("Left Lift: ", armL.getCurrentPosition());
            telemetry.addData("Right Lift: ", armR.getCurrentPosition());
            telemetry.addData("ANGLE: ", ANGLE.getCurrentPosition());
            telemetry.addData("Gyro: ", botAngle);
            telemetry.update();

        }
    }
    @Override
    protected void end() {

    }
}
