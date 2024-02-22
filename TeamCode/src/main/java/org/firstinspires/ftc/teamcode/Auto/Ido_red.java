package org.firstinspires.ftc.teamcode.Auto;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.OpMode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.dashboard.config.Config;

import org.openftc.apriltag.AprilTagDetection;

@Config
@Autonomous
public class Ido_red extends OpMode {

    @Override
    protected void run() {

        runtime.reset();
        if (left_middle_right_red  == 2){
            driveTo(4);
            sleep(5);

            turnToGyro(180);
            Ching_chung();
            sleep(5);

            driveTo(-10.3);
            sleep(5);

            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            IntakePower(500,1);
            sleep(5);

            AngleLift2(800,-1);
            sleep(5);

            driveTo(1);
            sleep(5);

            turnToGyro(87);
            sleep(200);

            driveTo(-16);
            sleep(5);

            driveTo(-14);
            sleep(2000);


            servo_R(1,1);
            sleep(1000);
            Elevator(1400);
            sleep(2);

            driveTo(4);
            sleep(2000);

            turnToGyro(90);
            sleep(5);
            ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            AngleLift2(-702,1);

            sleep(5);
            Elevator(10);
            sleep(5);


        }
        else if (left_middle_right_red  == 3){
            driveTo(15);
            sleep(5);

            turnToGyro(90);
            Ching_chung();
            sleep(5);

            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            IntakePower(500,1);
            sleep(5);

            AngleLift2(800,-1);
            sleep(5);

            driveTo(1);
            sleep(5);

            turnToGyro(180);
            sleep(200);

            driveTo(-16);
            sleep(5);

            driveTo(-14);
            sleep(2000);


            servo_R(1,1);
            sleep(1000);
            Elevator(1400);
            sleep(2);

            driveTo(4);
            sleep(2000);

            turnToGyro(90);
            sleep(5);
            ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            AngleLift2(-702,1);

            sleep(5);
            Elevator(10);
            sleep(5);

        }
        else if (left_middle_right_red  == 1) {
            driveTo(5);
            sleep(5);

            turnToGyro(90);
            turnToGyro(90);
            turnToGyro(50);
            Ching_chung();
            sleep(5);

            driveTo(-8);
            sleep(5);

            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            IntakePower(500,1);
            sleep(5);

            AngleLift2(800,-1);
            sleep(5);

            driveTo(1);
            sleep(200);
            turnToGyro(60);
            sleep(200);

            SideWalk(-5);
            sleep(200);

            driveTo(-20);
            sleep(200);
            SideWalk(12);
            SideWalk(-4);
            SideWalk(1);
            sleep(200);
            driveTo(-9);
            sleep(200);

            servo_R(1,1);
            sleep(1000);
            Elevator(1400);
            sleep(2);

            turnToGyro(90);
            sleep(5);
            ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            AngleLift2(-702,1);

            sleep(5);
            Elevator(10);
            sleep(5);

        }


    }


    @Override
    protected void end() {

    }

    public void Find(){

        AprilTagDetection tagOfInterest = null;

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                camera.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (opModeIsActive()){

            if(tagOfInterest == null || X_Value < 2) {
                   // drivetrain.moveTo(10);
                }

            telemetry.addData("X_Value: ",X_Value);
            telemetry.update();

        }

    }

    public void Ching_chung(){

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AngleLift2(-738,1);
        ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        IntakePower(-500,1);

        Elevator(727);

        AngleLift2(685,-1);
        ANGLE.setPower(0);

        Elevator(722);

        IntakePower(2000,1);
        servo_L(2,-1);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakePower(-350,-1);

        Elevator(1300);

        AngleLift2(-702, 1);

        ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void AngleLift2(int position, double power){
        int wanted_position = position + ANGLE.getCurrentPosition();

        ANGLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ANGLE.setTargetPosition(wanted_position);
        ANGLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && ANGLE.isBusy()){
            ANGLE.setPower(power);
            idle();
        }
        ANGLE.setPower(0);

    }

    public void AngleReset(double power) {

        int wanted_position = -1 * ANGLE.getCurrentPosition();

        ANGLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ANGLE.setTargetPosition(wanted_position);
        ANGLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && ANGLE.isBusy()) {
            ANGLE.setPower(power);
            idle();
        }
        ANGLE.setPower(0);

    }

    public void fix(){
        while (opModeIsActive()){
            double ticks_per_cm = 537.6 / 4;

            ticks_per_cm = ticks_per_cm * ANGLE.getCurrentPosition();

            double wanted_position = Math.cos(ticks_per_cm / 6);
            telemetry.addData("Say", "Hello Drive");
            telemetry.addData(" ticks_per_cm: ", ticks_per_cm);
            telemetry.addData("wantedPose: ", wanted_position);
            telemetry.update();
        }
    }
}

