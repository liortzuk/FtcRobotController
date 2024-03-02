package org.firstinspires.ftc.teamcode.Elevator;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID;

public class Elevator {

    ElapsedTime runtime = new ElapsedTime();
    private Servo trigger, angle;
    private CRServo LeftServo, RightServo;
    private Telemetry telemetry;
    private DcMotorEx  armR, armL, intake, ANGLE;

    public Elevator(DcMotorEx armL, DcMotorEx armR, DcMotorEx intake, DcMotorEx ANGLE, CRServo leftServo, CRServo rightServo, Servo trigger, Servo angle, Telemetry telemetry) {
       this.armL = armL;
       this.armR = armR;
       this.intake = intake;
       this.ANGLE = ANGLE;
       this.LeftServo = leftServo;
       this.RightServo = rightServo;
       this.trigger = trigger;
       this.angle = angle;
       this.telemetry = telemetry;
    }


    public void Elevator(double position){

        double positionWanted = position;
        PID pid = new PID(0.01, 0, 0, 0, 0);
            while (Math.abs(position) != Math.abs(armL.getCurrentPosition())) {
                pid.setWanted(position);
                armR.setPower(pid.update(armL.getCurrentPosition()));
                armL.setPower(pid.update(armL.getCurrentPosition()));
            }
            armR.setPower(0);
            armL.setPower(0);
        }

    public void AngleLift(int position, double power){
        ANGLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ANGLE.setTargetPosition(position);
        ANGLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (ANGLE.isBusy()){
            ANGLE.setPower(power);
        }
        ANGLE.setPower(0);
        ANGLE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    public void IntakePower(int position, double power){
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setTargetPosition(position);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (intake.isBusy()){
            intake.setPower(power);
            LeftServo.setPower(.7);
            RightServo.setPower(-.2);

        }
        LeftServo.setPower(0);
        RightServo.setPower(0);
        intake.setPower(0);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      }

    public void servo_R(double seconds, double power){
        runtime.reset();

        while (seconds > runtime.seconds()){
            RightServo.setPower(power);
        }RightServo.setPower(0);
    }

    public void servo_L(double seconds, double power){
        runtime.reset();

        while (seconds > runtime.seconds()){
            LeftServo.setPower(power);
        }LeftServo.setPower(0);
    }

    public void Ching_chung(){

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AngleLift(-738,1);
        ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        IntakePower(-500,1);

        Elevator(727);

        AngleLift(685,-1);
        ANGLE.setPower(0);

        Elevator(722);

        IntakePower(2000,1);
        servo_L(2,-1);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakePower(-350,-1);

        Elevator(1300);

        AngleLift(-702, 1);

        ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    }

