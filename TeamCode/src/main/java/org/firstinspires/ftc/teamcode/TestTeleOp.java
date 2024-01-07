package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TestTeleOp extends OpMode {

    float angle;
    double linearPosition;
    double linearDelta;

    @Override
    protected void postInit() {

    }

    @Override
    public void run(){
        runtime.reset();
        telemetry.addData("Say", "Hello Drive");

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);



        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;
            double drift = gamepad1.left_stick_x;
            double slide = gamepad1.right_stick_x;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            //Roni is the only way you can move forward
            Roni(forward, drift, slide, botHeading);

            if(gamepad1.left_trigger != 0){
                power(-gamepad1.left_trigger);
            }else if(gamepad1.right_trigger != 0){
                power(gamepad1.right_trigger);
            }



            if (gamepad2.x) {
               LeftServo.setPower(1);
               RightServo.setPower(-1);
            } else if (gamepad2.y) {
                LeftServo.setPower(-1);
                RightServo.setPower(1);
            }else {
                LeftServo.setPower(0);
                RightServo.setPower(0);
            }

            if(gamepad2.left_bumper){
                LeftServo.setPower(-1);
            }
            else if(gamepad2.right_bumper){
                RightServo.setPower(1);
            }

            if (gamepad2.left_stick_y != 0) {
                armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                armR.setPower(-gamepad2.left_stick_y / 2);
                armL.setPower(-gamepad2.left_stick_y);
            }else if (armL.getCurrentPosition() > 2400){
                armR.setPower(0);
                armL.setPower(0);
            } else {
                armR.setPower(0);
                armL.setPower(0);
            }


            ///take in
            if(gamepad2.b){
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intake.setPower(.5);
            }
            ///put out
            else if(gamepad2.a){
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intake.setPower(-.5);
            }
            else {
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intake.setPower(0);
            }


            if(gamepad2.left_trigger !=0){
                ANGLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ANGLE.setPower(.5);
            } else if (gamepad2.right_trigger != 0) {
                ANGLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ANGLE.setPower(-.5);
            }
            else {
                ANGLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ANGLE.setPower(0);
            }


            if(gamepad2.x && gamepad2.y){

                LeftServo.setPower(0);
                RightServo.setPower(0);

                armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Elevator(756,758, 1,1);
                armR.setPower(0);
                armL.setPower(0);

                AngleLift(704,-1);
                ANGLE.setPower(0);

                Elevator(-1104,705, .5,.5);
                armR.setPower(0);
                armL.setPower(0);

                IntakePower(340,1);

                for(int i =0 ; i < 100000; i++){
                    LeftServo.setPower(-1);
                    RightServo.setPower(1);
                }

            }else if (touch.isPressed()) {
                armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }





            if (gamepad1.options)



            telemetry.addData("ARM Right", armR.getCurrentPosition());
            telemetry.addData("ARM Left", armL.getCurrentPosition());
            telemetry.addData("ANGLE", ANGLE.getCurrentPosition());
            telemetry.addData("Right Power", armR.getVelocity());
            telemetry.addData("Left Power", armL.getVelocity());
            telemetry.addData("intake position",intake.getCurrentPosition());
            telemetry.update();

        }

    }


    private void Elevator(int Right_Target, int Left_Target, double PowerR, double PowerL){

        armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armR.setTargetPosition(Right_Target);
        armL.setTargetPosition(Left_Target);

        armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armR.setPower(PowerR);
        armL.setPower(PowerL);

        while (opModeIsActive()  && armL.isBusy() && armR.isBusy()){
            idle();
        }
    }

    private void AngleLift(int position, double power){
        ANGLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ANGLE.setTargetPosition(position);
        ANGLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ANGLE.setPower(power);

        while (opModeIsActive() && ANGLE.isBusy()){
            idle();
        }
    }

    private void IntakePower(int position, double power){
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setTargetPosition(position);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake.setPower(power);

        while (opModeIsActive()  && intake.isBusy()){
            idle();
        }
    }

    @Override
    protected void end() {



    }
}
