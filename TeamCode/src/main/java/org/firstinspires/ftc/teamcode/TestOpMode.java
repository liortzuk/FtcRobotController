package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TestOpMode extends OpMode {

    float angle;

    @Override
    protected void postInit() {

    }

    @Override
    public void run() {
        runtime.reset();

        angle = getGyro();

        telemetry.addData("Say", "Hello Drive");
        while (opModeIsActive()) {

            double turn = (gamepad1.right_trigger - gamepad1.left_trigger) * .5;
            ///movement
            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                if (turn == 0)
                    lockedAngleMoveFreeWithGyro(gamepad1.left_stick_x * .75, gamepad1.left_stick_y * .75, angle);
                else {
                    moveFreeWithGyro(gamepad1.left_stick_x * .75, gamepad1.left_stick_y * .75, turn);
                    angle = getGyro();
                }
            } else if (turn != 0) {
                move(0, turn * 3, false);
                angle = getGyro();
            }

            ///elevator power
                arm1.setPower(gamepad2.right_stick_y*.5);
                arm2.setPower(gamepad2.right_stick_y*.5);




            ///take in
            if(gamepad2.b == true){
                intake.setPower(.5);
            }
            ///put out
            else if(gamepad2.a == true){
                intake.setPower(-.5);
            }
            else {
                intake.setPower(0);
            }


            if(gamepad2.y == true){
                intake2.setPower(.5);
            } else if (gamepad2.x == true) {
                intake2.setPower(-.5);
            }
            else {
                intake2.setPower(0);
            }

            if(gamepad2.left_trigger !=0){
                ANGLE.setPower(.5);
            } else if (gamepad2.right_trigger != 0) {
                ANGLE.setPower(-.5);
            }
            else {
                ANGLE.setPower(0);
            }

            ///max
            if(gamepad2.a && gamepad2.b){
                arm1.setTargetPosition(2500);
                arm2.setTargetPosition(2500);

                while(arm1.getCurrentPosition() != arm1.getTargetPosition() && arm2.getCurrentPosition() != arm2.getTargetPosition()){
                    arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm1.setPower(.5);
                    arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm2.setPower(.5);
                }
                arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad2.y && gamepad2.x){
                arm1.setTargetPosition(0);
                arm2.setTargetPosition(0);

                while(arm1.getCurrentPosition() != arm1.getTargetPosition() && arm2.getCurrentPosition() != arm2.getTargetPosition()){

                    arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm1.setPower(.5);
                    arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm2.setPower(.5);

                    if (touch.isPressed()) {
                        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }


            }





            if (gamepad1.options)
                calibrateGyro();


            telemetry.addData("Gyro", "%.2f", -getGyro());
            telemetry.addData("ARM1", arm1.getCurrentPosition());
            telemetry.addData("ARM2", arm2.getCurrentPosition());
            telemetry.update();



        }

    }

    @Override
    protected void end() {



    }
}
