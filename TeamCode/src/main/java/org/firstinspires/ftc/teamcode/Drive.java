
package org.firstinspires.ftc.teamcode;
import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Drive", group=" ")
public class Drive extends OpMode{

    /**
     * class must extend "LinearOpMode" in order to apply telemetry
     * also to apply other classes such as "gamepad" and hardwaremap
     **/

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor intake;

    private VisionPortal camera;

    private AprilTagProcessor atagProcessor;

    private boolean correct = false;

    private int targetId = 586;

    private double detYaw;
    private double detX;
    private double detY;

    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        atagProcessor = new AprilTagProcessor.Builder().build();

        camera = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class,"logi"))
                .addProcessor(atagProcessor)
                .build();


        intake = hardwareMap.get(DcMotor.class, "intake");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
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

    }
    @Override
    public void loop(){
        List<AprilTagDetection> currentDetections = atagProcessor.getDetections();

        // Get the joystick values
        float leftStickY = -gamepad1.left_stick_y;
        float leftStickX = gamepad1.left_stick_x;
        float rightStickX = gamepad1.right_stick_x;
        boolean rightBumper = gamepad1.right_bumper;

        // Cap the motor powers
//        frontLeftPower = Range.clip(frontLeftPower, -1, 1);
//        backLeftPower = Range.clip(backLeftPower, -1, 1);
//        frontRightPower = Range.clip(frontRightPower, -1, 1);
//        backRightPower = Range.clip(backRightPower, -1, 1);

        if(gamepad2.left_bumper){
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

            // Set the motor powers
            leftFront.setPower((leftStickX + leftStickY + rightStickX) / (rightBumper ? 3 : 1));
            leftBack.setPower((leftStickX - leftStickY - rightStickX) / (rightBumper ? 3 : 1));
            rightFront.setPower((leftStickX - leftStickY + rightStickX) / (rightBumper ? 3 : 1));
            rightBack.setPower((leftStickX + leftStickY - rightStickX) / (rightBumper ? 3 : 1));
        if( gamepad1.left_bumper){ correct = true; }
        if(correct){
            for(AprilTagDetection detection : currentDetections){
                if(detection.id == targetId){
                    detYaw = detection.ftcPose.yaw;
                    detX = detection.ftcPose.x;
                    detY = detection.ftcPose.y;
                    if(detX > 2){
                        leftFront.setPower(-0.3);
                        leftBack.setPower(0.3);
                        rightFront.setPower(0.3);
                        rightBack.setPower(-0.3);

                    } else if(detX < -2){

                        leftFront.setPower(0.3);
                        leftBack.setPower(0.3);
                        rightFront.setPower(-0.3);
                        rightBack.setPower(-0.3);

                    } else{

                        leftFront.setPower(0);
                        leftBack.setPower(0);
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                    }
                }
            }
        }

        telemetryAprilTag();

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
    @Override
    public void stop(){
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        if(camera.getCameraState() == VisionPortal.CameraState.STREAMING){
            camera.stopLiveView();
            camera.stopStreaming();
        }
    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = atagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }









}
