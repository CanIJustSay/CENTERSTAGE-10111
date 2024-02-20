
package org.firstinspires.ftc.teamcode;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
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
public class Drive extends OpMode{
    private ElapsedTime runtime = new ElapsedTime();

    private PIDController controller;

    public static double p = 0.013, i = 0, d = 0.001;    //PID gains to be tuned

    public static double f = 0.16;
    public static int target = 0;    //target position for the arm
    public static double tick_in_degrees = 537.6 / 360;

    private DcMotorEx arm;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx flip;
    private CRServo launcher;
    private Servo claw;
    private DcMotorEx arm1;

    boolean useManual = true;



    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        controller = new PIDController(p,i,d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftBack  = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        flip = hardwareMap.get(DcMotorEx.class,"flip");

        claw = hardwareMap.get(Servo.class,"claw");

        launcher = hardwareMap.get(CRServo.class,"launcher");

        arm = hardwareMap.get(DcMotorEx.class,"arm");
        arm1 = hardwareMap.get(DcMotorEx.class,"arm1");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);




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

        flip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void loop() {

        float leftStickY = -gamepad1.left_stick_y;
        float leftStickX = gamepad1.left_stick_x;
        float rightStickX = gamepad1.right_stick_x;
        boolean rightBumper = gamepad1.right_bumper;
        boolean y = gamepad1.y;

        //Driver 1 - Driving Mech
        {
            //Wheel movement
            {
                //drive
                if (gamepad1.left_bumper) {
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                } else {
                    leftFront.setPower((leftStickY + leftStickX + rightStickX) / (rightBumper ? 3 : 1));
                    leftBack.setPower((leftStickY - leftStickX + rightStickX) / (rightBumper ? 3 : 1));
                    rightFront.setPower((leftStickY - leftStickX - rightStickX) / (rightBumper ? 3 : 1));
                    rightBack.setPower((leftStickY + leftStickX - rightStickX) / (rightBumper ? 3 : 1));
                }
            }


        }

        if(y){
            launcher.setPower(-1);

        }


        //Driver 2 - Scoring Mech
        {
            //arm mech
            {
                controller.setPID(p,i,d);
                int flipPos = flip.getCurrentPosition();
                double pid = controller.calculate(flipPos,target);
                double ff = Math.cos(Math.toRadians(target / tick_in_degrees)) * f;

                double power = pid + ff;

               // flip.setPower(power);

                if((gamepad2.left_stick_y > 0 || gamepad2.left_stick_y < 0)){
                    arm.setPower(-gamepad2.left_stick_y);
                    arm1.setPower(-gamepad2.left_stick_y);
                } else if(gamepad2.left_stick_y == 0) {
                    arm.setPower(0.1);
                   arm1.setPower(0.1);
                }
                if(gamepad2.right_stick_y != 0){ useManual = true; }
                if(gamepad2.y){ useManual = false; target = 170; }

                if(useManual) flip.setPower(-gamepad2.right_stick_y / 3);

                if(!useManual) flip.setPower(power);
            }

            if(gamepad2.right_bumper){
                claw.setPosition(0.35);
            }
            else {
                claw.setPosition(0.85);
            }

        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

    }
    @Override
    public void stop(){
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

}
