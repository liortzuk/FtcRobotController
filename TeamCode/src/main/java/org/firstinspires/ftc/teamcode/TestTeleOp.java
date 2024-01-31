package org.firstinspires.ftc.teamcode;

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

    }

    @Override
    public void run(){

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


         double positionWanted = 0;
         PID pid = new PID(0.01, 0, 0, 0, 0);

        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;//-1 to 1
            double drift = gamepad1.left_stick_x;
            double slide = gamepad1.right_stick_x;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            //Roni is the only way you can move
            Roni(forward, drift, slide, botHeading);

                //lift power
            positionWanted += -gamepad2.left_stick_y * 30;

            if (positionWanted >= 2400){
                positionWanted = 2400;
            }else  if (positionWanted <= 0){
                positionWanted = 0;
            }

            pid.setWanted(positionWanted);
            armR.setPower(pid.update(-armR.getCurrentPosition()));
            armL.setPower(pid.update(armL.getCurrentPosition()));



            if (gamepad1.options) {
                imu.resetYaw();
            }


            if(gamepad2.left_bumper){
                LeftServo.setPower(-1);
            }
            else if(gamepad2.right_bumper){
                RightServo.setPower(1);
            }else {
                RightServo.setPower(0);
                LeftServo.setPower(0);
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

            if (gamepad2.x){
                RightServo.setPower(-1);
                ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamepad2.y){
                LeftServo.setPower(1);
            }

            if(gamepad2.dpad_up){
               // LeftServo.setPower(0);
                // RightServo.setPower(0);

                armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Elevator(782,782,1,1);

                AngleLift(728,-1);
                ANGLE.setPower(0);

                armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            telemetry.addData("Left Lift: ", armL.getCurrentPosition());
            telemetry.addData("Right Lift: ", armR.getCurrentPosition());
            telemetry.addData("ANGLE: ", ANGLE.getCurrentPosition());
            telemetry.update();

        }

    }
    @Override
    protected void end() {

    }
}
