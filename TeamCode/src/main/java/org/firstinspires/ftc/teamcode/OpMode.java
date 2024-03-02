package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YXZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.asin;
import static java.lang.Math.atan;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;
import static java.lang.StrictMath.toDegrees;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import android.util.Log;
import android.util.Size;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.OpenCV.Belman_camera;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID;

import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


public abstract class OpMode extends LinearOpMode {

    protected TouchSensor touch;
    protected Servo trigger, angle;
    protected CRServo LeftServo, RightServo;
    protected DcMotorEx DriveFrontLeft, DriveFrontRight, DriveBackLeft, DriveBackRight, armR, armL, intake, ANGLE;
    protected ElapsedTime runtime = new ElapsedTime();
    protected float gyroCalibration = 0;
    protected BNO055IMU imu;

    protected IMU Imu;
    public DriveTrain drivetrain;
    FtcDashboard dashboard;

    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    void initialize() {
        Side side;

        DriveFrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        DriveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        DriveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveFrontRight = hardwareMap.get(DcMotorEx.class, "FR");
        DriveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        DriveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveBackLeft = hardwareMap.get(DcMotorEx.class, "BL");
        DriveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        DriveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveBackRight = hardwareMap.get(DcMotorEx.class, "BR");
        DriveBackRight.setDirection(DcMotor.Direction.REVERSE);
        DriveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armR = hardwareMap.get(DcMotorEx.class,"ELEVATOR R");
        armR.setDirection(DcMotorEx.Direction.FORWARD);
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        armL = hardwareMap.get(DcMotorEx.class,"ELEVATOR L");
        armL.setDirection(DcMotorEx.Direction.FORWARD);
        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotorEx.class,"WHEELS");
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        ANGLE = hardwareMap.get(DcMotorEx.class,"ANGLE");
        ANGLE.setDirection(DcMotorEx.Direction.FORWARD);
        ANGLE.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        LeftServo = hardwareMap.get(CRServo.class, "Left Servo");
        RightServo = hardwareMap.get(CRServo.class, "Right Servo");

        angle = hardwareMap.get(Servo.class, "angle");
        trigger = hardwareMap.get(Servo.class, "trigger");


        //initCam();
        //initTfod();

      //  touch = hardwareMap.get(TouchSensor.class, "touch");



        // Retrieve the IMU from the hardware map
        Imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot in our case it's UP and LEFT
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        Imu.initialize(parameters);

        Imu.resetYaw();

       // Find();

    }


    @Override
    public void runOpMode() throws InterruptedException  {
        initialize();
        waitForStart();
        postInit();

        dashboard = FtcDashboard.getInstance();
        drivetrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu, this);

        if (opModeIsActive()) {
            run();
        }

        end();
    }

    protected void postInit() {

    }
    protected abstract void run();

    protected abstract void end();


    public void Roni(double y, double x, double rx, double botHeading){

        DriveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY - rotX - rx) / denominator;
        double backLeftPower = (rotY + rotX - rx) / denominator;
        double frontRightPower = (rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX + rx) / denominator;

        DriveFrontLeft.setPower(frontLeftPower);
        DriveBackLeft.setPower(backLeftPower);
        DriveFrontRight.setPower(frontRightPower);
        DriveBackRight.setPower(backRightPower);

    }

    public void Elevator(double position){

        double positionWanted = position * 100;
        PID pid = new PID(0.01, 0, 0, 0, 0);

        armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (position > armL.getCurrentPosition()){
            while (position > armL.getCurrentPosition()){
                pid.setWanted(positionWanted);
                armR.setPower(pid.update(armL.getCurrentPosition()));
                armL.setPower(pid.update(armL.getCurrentPosition()));
                positionWanted = positionWanted - armL.getCurrentPosition();
            }
            armR.setPower(0);
            armL.setPower(0);
        }else if (position < armL.getCurrentPosition()){
            double wantedSpeed = armL.getCurrentPosition();
            positionWanted = wantedSpeed * 100;
            while (position < armL.getCurrentPosition()){
                pid.setWanted(positionWanted);
                armR.setPower(pid.update(armL.getCurrentPosition()));
                armL.setPower(pid.update(armL.getCurrentPosition()));
                positionWanted = wantedSpeed - armL.getCurrentPosition();
            }
            armR.setPower(0);
            armL.setPower(0);
        }
    }

    public void Elevator2(double position){

        double positionWanted = position;
        PID pid = new PID(0.01, 0, 0, 0, 0);


        if (position > armL.getCurrentPosition()) {
            while (position > armL.getCurrentPosition()) {
                pid.setWanted(position);
                armR.setPower(pid.update(armL.getCurrentPosition()));
                armL.setPower(pid.update(armL.getCurrentPosition()));
                // positionWanted = position - Math.abs(armL.getCurrentPosition());
                telemetry.addData("hi", armL.getPower());
                telemetry.update();
            }
            armR.setPower(0);
            armL.setPower(0);
           }else if (position < armL.getCurrentPosition()){
              double wantedSpeed = armL.getCurrentPosition();
              positionWanted = wantedSpeed;
              while (position < armL.getCurrentPosition()){
              pid.setWanted(position);
              armR.setPower(pid.update(armL.getCurrentPosition()));
              armL.setPower(pid.update(armL.getCurrentPosition()));

           }
            armR.setPower(0);
            armL.setPower(0);
        }
    }

    public void AngleLift(int position, double power){
        ANGLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ANGLE.setTargetPosition(position);
        ANGLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && ANGLE.isBusy()){
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


        while (opModeIsActive()  && intake.isBusy()){
            intake.setPower(power);
            LeftServo.setPower(.7);
            RightServo.setPower(-.2);

        }
        LeftServo.setPower(0);
        RightServo.setPower(0);
        intake.setPower(0);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double xPosition_red;
    public double yPosition_red;
    public double xPosition_blue;
    public double yPosition_blue;

    public int left_middle_right_red = 2;
    public int left_middle_right_blue = 2;

    public double X_Value = 0;
    public OpenCvCamera camera;

    public AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public void initCam(){

        // Lens intrinsics
        // UNITS ARE PIXELS
        // Note:  this calibration is for the C921HD webcam at 1920x1080.
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;

        int ID_TAG_OF_INTEREST0 = 0; // Tag ID 0 from the 36h11 family
        int ID_TAG_OF_INTEREST1 = 1;
        int ID_TAG_OF_INTEREST2 = 2;

        int ID_TAG_OF_INTEREST3 = 3;
        int ID_TAG_OF_INTEREST4 = 4;
        int ID_TAG_OF_INTEREST5 = 5;


        AprilTagDetection tagOfInterest = null;


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cum"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

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

        while (!isStarted() && !isStopRequested() || opModeInInit())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_TAG_OF_INTEREST0 || tag.id == ID_TAG_OF_INTEREST1 || tag.id ==  ID_TAG_OF_INTEREST2 || tag.id == ID_TAG_OF_INTEREST4 || tag.id == ID_TAG_OF_INTEREST5 || tag.id == ID_TAG_OF_INTEREST3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    xPosition_red = 10 * tagOfInterest.pose.x;
                    xPosition_red  = (int)xPosition_red  * 10;

                    yPosition_red  = 10 * tagOfInterest.pose.y;
                    yPosition_red  = (int)yPosition_red  * 10;

                    //left
                    if (Math.abs(xPosition_red ) > 90){
                        left_middle_right_red  = 1;
                    } else if (xPosition_red  < 0) {
                        left_middle_right_red  = 3;
                    } else {
                        left_middle_right_red  = 2;
                    }

                    xPosition_blue = 10 * tagOfInterest.pose.x;
                    xPosition_blue = (int)xPosition_blue * 10;

                    yPosition_blue = 10 * tagOfInterest.pose.y;
                    yPosition_blue = (int)yPosition_blue * 10;

                    //left
                    if (Math.abs(xPosition_blue) > 150){
                        left_middle_right_blue = 3;
                    } else if (xPosition_blue < 60) {
                        left_middle_right_blue = 1;
                    } else {
                        left_middle_right_blue = 2;
                    }
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    telemetry.addData("Y Axes: ", yPosition_blue);
                    telemetry.addData("X Axes: ", xPosition_blue);
                    telemetry.addData("left center right: ", left_middle_right_blue);

                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    telemetry.addData("Y Axes: ", yPosition_red);
                    telemetry.addData("X Axes: ", xPosition_red);
                    telemetry.addData("left center right: ", left_middle_right_red);
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");

                    telemetry.update();
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
            telemetry.addLine("Waiting for start");
            telemetry.update();
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));

        X_Value += rot.firstAngle;

    }

    public void turnToGyro(double degrees){

        Imu.resetYaw();

        double error = degrees + Math.abs(Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
       // PID_turn pid_turn = new PID_turn(error,5,4,0);

        PID pid = new PID(.5,.2,0,0,0);



        if (degrees > 0) {
            while (Math.abs(error) > 15) {
                double botAngle = Math.abs(Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

                double ticks_per_cm = 537.6 / 9.6;

                double wantedPose = ticks_per_cm*degrees;

                DriveFrontLeft.setPower(-75);
                DriveFrontRight.setPower(75);

                DriveBackRight.setPower(75);
                DriveBackLeft.setPower(-75);
                error = degrees - botAngle;

            }
            DriveFrontLeft.setPower(0);
            DriveFrontRight.setPower(0);
            DriveBackRight.setPower(0);
            DriveBackLeft.setPower(0);
        }
        else if (degrees < 0) {
            while (error < -15) {
                double botAngle = Math.abs(Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));


                DriveFrontLeft.setPower(75);
                DriveFrontRight.setPower(-75);

                DriveBackRight.setPower(-.75);
                DriveBackLeft.setPower(75);
                error = degrees + botAngle;

            }
            DriveFrontLeft.setPower(0);
            DriveFrontRight.setPower(0);
            DriveBackRight.setPower(0);
            DriveBackLeft.setPower(0);

        }

    }

    public void driveTo(double cm){
        DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double positionWanted = 0;
        PID pid = new PID(0.5, 0.2, 0.1, 0, 0);

        double ticks_per_cm = 537.6 / 9.6;
        positionWanted = cm * ticks_per_cm;
        pid.setWanted((int)positionWanted);

        while (Math.abs(positionWanted)> Math.abs(DriveFrontLeft.getCurrentPosition()) + 2){
            DriveFrontLeft.setPower(pid.update(DriveFrontLeft.getCurrentPosition()) / 2);
            DriveBackLeft.setPower(pid.update(DriveFrontLeft.getCurrentPosition()) / 2);

            DriveFrontRight.setPower(pid.update(DriveFrontLeft.getCurrentPosition()));
            DriveBackRight.setPower(pid.update(DriveFrontLeft.getCurrentPosition()));
        }

        DriveFrontLeft.setPower(0);
        DriveBackLeft.setPower(0);

        DriveFrontRight.setPower(0);
        DriveBackRight.setPower(0);

        DriveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    public void SideWalk(double cm){
        DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double positionWanted = 0;
        PID pid = new PID(0.5, 0.5, 0.2, 0, 0);

        double ticks_per_cm = 537.6 / 9.6;
        positionWanted = cm * ticks_per_cm;
        pid.setWanted((int)positionWanted);

        while (Math.abs(positionWanted) > Math.abs(DriveFrontLeft.getCurrentPosition())){
            DriveFrontLeft.setPower(-pid.update(DriveFrontLeft.getCurrentPosition()));
            DriveBackLeft.setPower(pid.update(DriveFrontLeft.getCurrentPosition()));

            DriveFrontRight.setPower(-pid.update(DriveFrontLeft.getCurrentPosition()));
            DriveBackRight.setPower(pid.update(DriveFrontLeft.getCurrentPosition()));
        }

        DriveFrontLeft.setPower(0);
        DriveBackLeft.setPower(0);

        DriveFrontRight.setPower(0);
        DriveBackRight.setPower(0);

        DriveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void initTfod() {


        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                //.setModelLabels()
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                  .setModelInputSize(1000)
                  .setModelAspectRatio(16.0 / 9.0)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "cum"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1920,1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

        while (runtime.seconds() < 20){
            telemetryTfod();
        }

    }   // end method initTfod()

    public boolean object = false;
    public void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position x / y: ", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.update();

            xPosition_red = recognition.getLeft() + recognition.getRight()/ 2;
            yPosition_red  = recognition.getTop()  + recognition.getBottom()/ 2;


            //left
            if (Math.abs(xPosition_red ) > 975){
                left_middle_right_red  = 1;
                visionPortal.stopStreaming();
            } else if (xPosition_red  < 888) {
                left_middle_right_red  = 3;
                visionPortal.stopStreaming();
            } else {
                left_middle_right_red  = 2;
                visionPortal.stopStreaming();
            }


        }
        if (currentRecognitions.size() == 0){
           object = false;
        }else {
            object = true;
        }

    }   // end method telemetryTfod()

    public void turnToGyroPower(double degrees,double power){

        Imu.resetYaw();

        double error = degrees + Math.abs(Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        // PID_turn pid_turn = new PID_turn(error,5,4,0);

        PID pid = new PID(.5,.2,0,0,0);



        if (degrees > 0) {
            while (Math.abs(error) > 15) {
                double botAngle = Math.abs(Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

                double ticks_per_cm = 537.6 / 9.6;

                double wantedPose = ticks_per_cm*degrees;

                DriveFrontLeft.setPower(-power);
                DriveFrontRight.setPower(power);

                DriveBackRight.setPower(power);
                DriveBackLeft.setPower(-power);
                error = degrees - botAngle;

            }
            DriveFrontLeft.setPower(0);
            DriveFrontRight.setPower(0);
            DriveBackRight.setPower(0);
            DriveBackLeft.setPower(0);
        }
        else if (degrees < 0) {
            while (error < -15) {
                double botAngle = Math.abs(Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));


                DriveFrontLeft.setPower(power);
                DriveFrontRight.setPower(-power);

                DriveBackRight.setPower(-power);
                DriveBackLeft.setPower(power);
                error = degrees + botAngle;

            }
            DriveFrontLeft.setPower(0);
            DriveFrontRight.setPower(0);
            DriveBackRight.setPower(0);
            DriveBackLeft.setPower(0);

        }

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

}

