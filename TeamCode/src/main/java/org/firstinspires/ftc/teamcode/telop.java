package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "TeleOp",group = "Linear OpMode")
//@disable

public abstract class telop extends OpMode{
    float angle;
    @Override
    protected void postInit() {
    }
    @Override
    public void run() {
        runtime.reset();

        angle = getGyro();

        telemetry.addData("Say", "Hello Drive");

        waitForStart();

        while (opModeIsActive()) {
            double turn = (gamepad1.right_trigger - gamepad1.left_trigger) * .5;

            if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
                if (turn == 0)
                    lockedAngleMoveFreeWithGyro(gamepad1.right_stick_x * .5, gamepad1.right_stick_y * .5, angle);
                else {
                    moveFreeWithGyro(gamepad1.right_stick_x * .5, gamepad1.right_stick_y * .5, turn * .5);
                    angle = getGyro();
                }
            } else if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                if (turn == 0)
                    lockedAngleMoveFreeWithGyro(gamepad1.left_stick_x * .75, gamepad1.left_stick_y * .75, angle);
                else {
                    moveFreeWithGyro(gamepad1.left_stick_x * .75, gamepad1.left_stick_y * .75, turn);
                    angle = getGyro();
                }
            } else if (turn != 0) {
                move(0, turn * .75, false);
                angle = getGyro();
            } else if (gamepad2.left_stick_x != 0 || gamepad2.left_stick_y != 0) {
                moveFreeWithGyro(gamepad2.left_stick_x * .5, gamepad1.left_stick_y * .5, gamepad2.right_trigger - gamepad2.left_trigger);
            } else {
                move(0, 0, true);
                angle = getGyro();
            }

            if (gamepad1.options)
                calibrateGyro();

            telemetry.addData("Gyro", "%.2f", -getGyro());
            telemetry.update();

        }

    }

    @Override
    protected void end() {

    }
}
