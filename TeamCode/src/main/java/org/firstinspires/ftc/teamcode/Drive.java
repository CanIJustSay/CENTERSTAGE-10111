
package org.firstinspires.ftc.teamcode;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name="Drive", group=" ")
/*
    TODO
        pixel detection?? for the stack in auto?
        idea:
            knock over pixel stack and intake the lowest one?
            vision to sense the knocked over pixels


 */
public class Drive extends OpMode{

    /**
     * class must extend "LinearOpMode" in order to apply telemetry
     * also to apply other classes such as "gamepad" and hardwaremap
     **/

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    IMU imu;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    private PIDController controller;

    public static double p = 0.056, i = 0, d = 0.001;    //PID gains to be tuned

    public static double f = 0.01;
    public static int target = 0;    //target position for the arm
    public static double tick_in_degrees = 537.6 / 360;
    private DcMotorEx arm; // not yet on the bot - we will use PIDF gains for this maybe???
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotor intake;

    private Servo wrist;
    private boolean enterCorrective;

    private VisionPortal camera;

    private AprilTagProcessor atagProcessor;

    private boolean correct = false;

    private int targetId = 586;

    private double detYaw;
    private boolean manual;
    private CRServo servo;
    private Servo launcher;

    private Servo knuckle;
    private double detX;
    private double detY;

    private DcMotor lift;

    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();


//        imu = hardwareMap.get(IMU.class,"imu");

        lift = hardwareMap.get(DcMotor.class,"lift");

        servo = hardwareMap.get(CRServo.class,"servo");

        controller = new PIDController(p,i,d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftBack  = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        arm = hardwareMap.get(DcMotorEx.class,"arm");

        launcher = hardwareMap.get(Servo.class,"launcher");

//        atagProcessor = new AprilTagProcessor.Builder().build();

        wrist = hardwareMap.get(Servo.class,"wrist");

        knuckle = hardwareMap.get(Servo.class,"knuckle");

//        camera = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class,"logi"))
//                .addProcessor(atagProcessor)
//                .build();

        intake = hardwareMap.get(DcMotor.class, "intake");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);

        manual = false;
    }
    @Override
    public void init_loop(){


    }
    @Override
    public void start(){
        runtime.reset();

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    /***
     *
     *

     * Intake = done
     * Lift   = done
     * Knuckle = done
     * Arm     = could be improved, but done
     * Wrist   = done
     * Drive   = done
     * Lift Servo = done
     * Launcher = done
     *
     * Auto:
     *  I have no clue. Should work, but you know.
     *
     */

    @Override
    public void loop(){


        float leftStickY = -gamepad1.left_stick_y;
        float leftStickX = gamepad1.left_stick_x;
        float rightStickX = gamepad1.right_stick_x;
        boolean rightBumper = gamepad1.right_bumper;



        if(gamepad1.left_trigger > 0){
            intake.setPower(1);
        } else if(gamepad1.right_trigger > 0) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        /**
         * fL = y+x+r
         * rL = y-x+r
         * fR = y-x-r
         * rR = y+x-r
         */

        //drive
        if(gamepad1.left_bumper) {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        } else {
            leftFront.setPower((leftStickY + leftStickX + rightStickX) / (rightBumper ? 3 : 1) );
            leftBack.setPower((leftStickY - leftStickX + rightStickX) / (rightBumper ? 3 : 1) );
            rightFront.setPower((leftStickY - leftStickX - rightStickX) / (rightBumper ? 3 : 1) );
            rightBack.setPower((leftStickY + leftStickX - rightStickX) / (rightBumper ? 3 : 1) );
        }


        lift.setPower(gamepad2.right_stick_y);

        //scissor lift
        if (gamepad1.a){
            servo.setPower(1);
        } else if (gamepad1.b) {
            servo.setPower(0);
        }

        knuckle.setPosition( gamepad2.dpad_up ? 0.8 : (gamepad2.dpad_left ? 0.5 : (gamepad2.dpad_right ? 0.26 : 0.35 ) ) );
        //wrist positioning
        if(gamepad2.a){
            wrist.setPosition(0);
        } else {
            wrist.setPosition(0.57);
        }

        //needs to stay level until told otherwise, as it goes up, the angle should change
        //consistenly with the wrist

        if(gamepad1.y){
            launcher.setPosition(0);
        }


        // target += (gamepad2.left_stick_y * 5);
        //arm pid
        controller.setPID(p,i,d);

        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(target / tick_in_degrees)) * f;

        double power;
      //  target = 100;
        power = pid + ff;

        arm.setPower(
                -gamepad2.left_stick_y
               // power
        );


        /**
         if(gamepad2.x && !manual){
         manual = true;
         } else if(gamepad2.x && manual){
         manual = false;
         }
         */

//
//        telemetry.addData("Current Pos", arm.getCurrentPosition());
//        telemetry.addData("Arm Pos",armPos);
//        telemetry.addData("Target", target);
//
//
//
//
//
//        telemetryAprilTag();
//
//        telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);
//
//        // Check to see if heading reset is requested
//        if (gamepad1.y) {
//            telemetry.addData("Yaw", "Resetting\n");
//            imu.resetYaw();
//        } else {
//            telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
//        }
//
//        // Retrieve Rotational Angles and Velocities
//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
//
//        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
//        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
//        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
//        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
//        telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
//        telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
//        telemetry.update();


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
    @Override
    public void stop(){
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

//        if(camera.getCameraState() == VisionPortal.CameraState.STREAMING){
//            camera.stopLiveView();
//            camera.stopStreaming();
//        }
    }










}
