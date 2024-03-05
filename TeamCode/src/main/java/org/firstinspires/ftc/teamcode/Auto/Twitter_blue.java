package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Elevator.Elevator;

@Autonomous
public class Twitter_blue extends OpMode {

    Elevator elevator = new Elevator(armL, armR, intake, ANGLE, LeftServo, RightServo, trigger, angle);
    @Override
    protected void run() {
        DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu);
        runtime.reset();

        if (left_middle_right_blue == 2){
            driveTrain.driveTo(4);
            sleep(5);

            driveTrain.turnToGyro(180);
            elevator.Ching_chung();
            sleep(5);

            driveTrain.driveTo(-9);
            sleep(5);

            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator.IntakePower(500,1);
            sleep(5);

            elevator.AngleLift(800,-1);
            sleep(5);

            driveTrain.driveTo(4);
            sleep(5);

            driveTrain.turnToGyro(-87);
            sleep(1000);

            driveTrain.driveTo(-16);
            sleep(5);

            driveTrain.SideWalk(-20);
            sleep(200);

            driveTrain.turnToGyro(-17);
            sleep(5);

            driveTrain.driveTo(-8);
            sleep(200);

            elevator.servo_R(1,1);
            sleep(1000);
            elevator.Elevator_function(1400);
            sleep(2);

            driveTrain.driveTo(4);
            sleep(200);

            driveTrain.turnToGyro(90);
            sleep(5);
            ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator.AngleLift(-702,1);

            sleep(5);
            elevator.Elevator_function(10);
            sleep(5);


        }
        else if (left_middle_right_blue  == 3){
            driveTrain.driveTo(15);
            sleep(5);

            driveTrain.turnToGyro(-90);
            elevator.Ching_chung();
            sleep(5);
            driveTrain.driveTo(-3);

            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator.IntakePower(500,1);
            sleep(5);
            driveTrain.driveTo(3);
            sleep(5);
            driveTrain.SideWalk(5);
            sleep(5);

            elevator.AngleLift(800,-1);
            sleep(5);

            driveTrain.driveTo(1);
            sleep(5);

            driveTrain.turnToGyro(-180);
            sleep(200);

            driveTrain.driveTo(-16);
            sleep(5);

            driveTrain.SideWalk(-6);
            sleep(100);

            driveTrain.driveTo(-14);
            sleep(2000);

            elevator.servo_R(1,1);
            sleep(1000);
            elevator.Elevator_function(1400);
            sleep(2);

            driveTrain.driveTo(4);
            sleep(2000);

            driveTrain.turnToGyro(-90);
            sleep(5);
            ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator.AngleLift(-702,1);

            sleep(5);
            elevator.Elevator_function(10);
            sleep(5);

        }
        else if (left_middle_right_blue  == 1) {
            driveTrain.driveTo(5);
            sleep(5);

            driveTrain.turnToGyro(-230);
            elevator.Ching_chung();
            sleep(5);

            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator.IntakePower(500,1);
            sleep(5);

            elevator.AngleLift(800,-1);
            sleep(5);

            driveTrain.driveTo(1);
            sleep(200);
            driveTrain.turnToGyro(-60);
            sleep(200);

            driveTrain.driveTo(-20);
            sleep(200);
            driveTrain.SideWalk(-6);
            sleep(200);
            driveTrain.driveTo(-5);
            sleep(200);

            elevator.servo_R(1,1);
            sleep(1000);
            elevator.Elevator_function(1400);
            sleep(2);

            driveTrain.turnToGyro(-90);
            sleep(5);
            ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator.AngleLift(-702,1);

            sleep(5);
            elevator.Elevator_function(10);
            sleep(5);

        }

    }

    @Override
    protected void end() {

    }

}
