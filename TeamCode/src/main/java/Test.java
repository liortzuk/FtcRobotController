import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.PID;


public class Test extends LinearOpMode {

    protected DcMotorEx armR, armL;

    Gamepad gamepad;
    @Override
    public void runOpMode() throws InterruptedException {
        armR = hardwareMap.get(DcMotorEx.class,"ELEVATOR R");
        armR.setDirection(DcMotorEx.Direction.FORWARD);
        armR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        armL = hardwareMap.get(DcMotorEx.class,"ELEVATOR L");
        armL.setDirection(DcMotorEx.Direction.FORWARD);
        armL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);



        PID pid = new PID(0.01, 0, 0, 0, 0);
        double positionWanted = 0;
        waitForStart();
        while (!isStopRequested()){


            positionWanted += -gamepad2.left_stick_y * 20;

            pid.setWanted(positionWanted);
            armR.setPower(pid.update(-armR.getCurrentPosition()));
            armL.setPower(pid.update(armL.getCurrentPosition()));

            telemetry.addData("armR: ", -armR.getCurrentPosition());
            telemetry.addData("armL: ", armL.getCurrentPosition());
            telemetry.update();
        }

    }

}
