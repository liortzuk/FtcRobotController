package org.firstinspires.ftc.teamcode.Elevator;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PID;

public class Elevator {

    ElapsedTime runtime = new ElapsedTime();
    private final Servo trigger, angle;
    private final CRServo LeftServo, RightServo;
    private final DcMotorEx  armR, armL, intake, ANGLE;

    public Elevator(DcMotorEx armL, DcMotorEx armR, DcMotorEx intake, DcMotorEx ANGLE, CRServo leftServo, CRServo rightServo, Servo trigger, Servo angle) {
       this.armL = armL;
       this.armR = armR;
       this.intake = intake;
       this.ANGLE = ANGLE;
       this.LeftServo = leftServo;
       this.RightServo = rightServo;
       this.trigger = trigger;
       this.angle = angle;
    }

     public void Elevator_function(int position){
        PID pid = new PID(0.01, 0, 0, 0, 0);

            while (Math.abs(position) > Math.abs(armL.getCurrentPosition())) {
                pid.setWanted(position);
                armR.setPower(pid.update(armL.getCurrentPosition()));
                armL.setPower(pid.update(armL.getCurrentPosition()));
            }
           armR.setPower(0);
           armL.setPower(0);
        }

    public void Elevator_function_down(int position){
        PID pid = new PID(0.01, 0, 0, 0, 0);

        while (Math.abs(position) < Math.abs(armL.getCurrentPosition())) {
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

    public void ResetAngle(){
        ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void reset_all(){
        ANGLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ANGLE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Ching_chung() {
        reset_all();

        AngleLift(-790, 1);
        IntakePower(-500, 1);

        Elevator_function(750);
        AngleLift(-65, 1);

        IntakePower(1000, -1);
        servo_R(1, -.15);

        Elevator_function(690);

        IntakePower(-500, 1);

        IntakePower(-350, -1);
        Elevator_function(1700);

        AngleLift(-800, 1);
        ResetAngle();
      }

      public void servo_left_and_right(double seconds, double power){
          runtime.reset();

          while (seconds > runtime.seconds()){
              RightServo.setPower(-power / 4);
              LeftServo.setPower(power);
          }LeftServo.setPower(0);
           RightServo.setPower(0);
      }

      public void daria(){
          ResetAngle();
          reset_all();

          Elevator_function(750);

          AngleLift(665,-1);

          Elevator_function(690);

          IntakePower(2500,1);
          servo_left_and_right(0.25,1);
          Elevator_function(1300);

          AngleLift(800,-1);

      }

      public void lihi(){
          AngleLift(0,1);
          Elevator_function_down(10);
      }
    }

