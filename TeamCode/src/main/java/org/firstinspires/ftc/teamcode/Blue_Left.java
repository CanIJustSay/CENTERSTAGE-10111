package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.CameraCode.PropDetectionPipeline.PropPositions.LEFT;
import static org.firstinspires.ftc.teamcode.CameraCode.PropDetectionPipeline.PropPositions.MIDDLE;
import static org.firstinspires.ftc.teamcode.CameraCode.PropDetectionPipeline.PropPositions.RIGHT;
import static org.firstinspires.ftc.teamcode.CameraCode.PropDetectionPipeline.PropPositions.UNFOUND;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CameraCode.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

import java.util.List;


@Autonomous(preselectTeleOp = "Drive")
public class Blue_Left extends OpMode {
    private VisionPortal visionPortal;
    private PropDetectionPipeline propProcessor;
    private SampleMecanumDrive drive;
    private AprilTagProcessor atagProcessor;
    private PIDController controller;

    public static double p = 0.007, i = 0, d = 0.0;

    public static double f = 0.16;
    public static int target = 1;    //target position for the arm

    public static double Ap = 0.014, Ai = 0, Ad = 0.00;    //PID gains to be tuned
    public static double Af = 0.26;
    public static int Atarget = 0;    //target position for the arm

    public static double tick_in_degrees = 537.6 / 360;
    private DcMotorEx flip;
    private DcMotorEx arm1;
    private DcMotorEx arm;
    private Servo claw;

    TrajectorySequence right;
    TrajectorySequence traj;
    TrajectorySequence left;


    @Override
    public void init() {

        // the current range set by lower and upper is the full range
        // HSV takes the form: (HUE, SATURATION, VALUE)
        // which means to select our colour, only need to change HUE
        // the domains are: ([0, 180], [0, 255], [0, 255])
        // this is tuned to detect red, so you will need to experiment to fine tune it for your robot
        // and experiment to fine tune it for blue

        arm = hardwareMap.get(DcMotorEx.class,"arm");
        arm1 = hardwareMap.get(DcMotorEx.class,"arm1");

        flip = hardwareMap.get(DcMotorEx.class,"flip");

        claw = hardwareMap.get(Servo.class, "claw");


        Scalar lower = new Scalar(0, 100, 100);
        Scalar upper = new Scalar(180, 255, 255);

        double minArea = 200; // the minimum area for the detection to consider for your prop
        drive = new SampleMecanumDrive(hardwareMap);
        controller = new PIDController(p,i,d);
        atagProcessor = new AprilTagProcessor.Builder().build();


        //to change what qualifies as middle, change the left and right dividing lines
        //picture it like this
        /*
        |----------|----------------|----------|
        |          |                |          |
        |          |                |          |
        |   LEFT   |     MIDDLE     |   RIGHT  |
        |          |                |          |
        |          |                |          |
        |          |                |          |
        |          |                |          |
        |          |                |          |
        |          |                |          |
        |          |                |          |
        |__________|________________|__________|

                   ^                ^
            this line              this line
            being 213              being 426

            Change these numbers accordingly to
            make you're middle larger or smaller
            depending on what you need/want
         */

        propProcessor = new PropDetectionPipeline(
                lower,
                upper,
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426 // the left dividing line, in this case the right third of the frame
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "logi")) // the camera on your robot is named "Webcam 1" by default
                .addProcessors(atagProcessor,propProcessor)
                .setCameraResolution(new Size(640, 480))
                .build();


        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Pose2d startPose = new Pose2d(14.5, 63.75, Math.toRadians(90.00));


        drive.setPoseEstimate(startPose);

        left = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{
                     claw.setPosition(0.85);
                })
                .setReversed(true)
                .lineTo(new Vector2d(26,39))
                .forward(5)
                .setReversed(false)
                .addTemporalMarker(()->{
                     target = 230;
                })
                .splineTo(new Vector2d(53,45),Math.toRadians(0))
                .addTemporalMarker(()->{
                    claw.setPosition(0.35);
                })
                .waitSeconds(2)
                .back(5)
                .addTemporalMarker(()->{
                     target = 0;
                })
                .lineToLinearHeading(new Pose2d(34.3,8.7, Math.toRadians(180)))
                .addDisplacementMarker(160,()->{
                    //raise the arm and open up the claw
                     arm1.setPower(0.49);
                     arm.setPower(0.49);
                })
                .addDisplacementMarker(166,()->{
                    arm1.setPower(0.1);
                    arm.setPower(0.1);
                })
                .lineTo(new Vector2d(-59,15.7))
                .addTemporalMarker(()->{
                     claw.setPosition(0.85);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(12.3,6.7))
                .addDisplacementMarker(230,()->{
                    arm1.setPower(-0.1);
                    arm.setPower(-0.1);
                })
                .lineToLinearHeading(new Pose2d(60,11, Math.toRadians(0)))
                .addTemporalMarker(()->{
                     claw.setPosition(0.35);
                })
                .back(5)
                .build();


        traj = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{
                    claw.setPosition(0.85);
                })
                .back(30)
                .setReversed(false)
                .addTemporalMarker(()->{
                    target = 230;
                })

                .splineTo(new Vector2d(53,38.4),Math.toRadians(0))
                .addTemporalMarker(()->{
                    claw.setPosition(0.35);
                })
                .waitSeconds(2)
                .back(5)
                .addTemporalMarker(()->{
                    target = 0;
                })
                .lineToLinearHeading(new Pose2d(34.3,8.7, Math.toRadians(180)))
                .addDisplacementMarker(160,()->{
                    //raise the arm and open up the claw
                    arm1.setPower(0.49);
                    arm.setPower(0.49);
                })
                .addDisplacementMarker(166,()->{
                    arm1.setPower(0.1);
                    arm.setPower(0.1);
                })
                .lineTo(new Vector2d(-59,15.7))
                .addTemporalMarker(()->{
                    claw.setPosition(0.85);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(12.3,6.7))
                .addDisplacementMarker(230,()->{
                    arm1.setPower(-0.1);
                    arm.setPower(-0.1);
                })
                .lineToLinearHeading(new Pose2d(60,11, Math.toRadians(0)))
                .addTemporalMarker(()->{
                    claw.setPosition(0.35);
                })
                .back(5)
                .build();


        right = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{
                    claw.setPosition(0.85);
                })
                .lineTo(new Vector2d(15.5,47.75))
                .setReversed(true)
                .splineTo(new Vector2d(10,39), Math.toRadians(180))
                .addTemporalMarker(()->{
                    target = 230;
                })
                .lineTo(new Vector2d(53,33))
                .addTemporalMarker(()->{
                    claw.setPosition(0.35);
                })
                .waitSeconds(2)
                .back(5)
                .addTemporalMarker(()->{
                    target = 0;
                })
                .lineToLinearHeading(new Pose2d(34.3,8.7, Math.toRadians(180)))
                .addDisplacementMarker(160,()->{
                    //raise the arm and open up the claw
                    arm1.setPower(0.49);
                    arm.setPower(0.49);
                })
                .addDisplacementMarker(166,()->{
                    arm1.setPower(0.1);
                    arm.setPower(0.1);
                })
                .lineTo(new Vector2d(-59,15))
                .addTemporalMarker(()->{
                    claw.setPosition(0.85);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(12.3,6.7))
                .addDisplacementMarker(230,()->{
                    arm1.setPower(-0.1);
                    arm.setPower(-0.1);
                })
                .lineToLinearHeading(new Pose2d(60,11, Math.toRadians(0)))
                .addTemporalMarker(()->{
                    claw.setPosition(0.35);
                })
                .back(5)
                .build();



    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", propProcessor.getRecordedPropPosition());
        telemetry.addData("Currently Detected Mass Center", "x: " + propProcessor.getLargestContourX() + ", y: " + propProcessor.getLargestContourY());
        telemetry.addData("Current contour area", propProcessor.getLargestContourArea());
        //telemetryAprilTag();

    }

    @Override
    public void start() {

/*
        TODO
              check up on rr - could start being dumb again, tuning wise
              widen left and right bounds because middle needs to be larger

 */
        PropDetectionPipeline.PropPositions recordedPropPosition = propProcessor.getRecordedPropPosition();

        if (propProcessor.getLargestContourArea() < 4000) {
            recordedPropPosition = RIGHT;
        }

        //right of the mat
        switch (recordedPropPosition) {

            case LEFT:
                drive.followTrajectorySequenceAsync(left);
                break;

            case MIDDLE:
                drive.followTrajectorySequenceAsync(traj);
                break;
            case RIGHT:
                drive.followTrajectorySequenceAsync(right);
                break;

        }
        propProcessor.close();
    }

    @Override
    public void loop() {
        // telemetryAprilTag();


        controller.setPID(p,i,d);
        int flipPos = flip.getCurrentPosition();
        double pid = controller.calculate(flipPos,target);
        double ff = Math.cos(Math.toRadians(target / tick_in_degrees)) * f;

        double power = pid + ff;

        controller.setPID(Ap,Ai,Ad);
        int armPos = arm1.getCurrentPosition();
        double Apid = controller.calculate(armPos,Atarget);
        double Aff = Math.cos(Math.toRadians(Atarget / tick_in_degrees)) * Af;

        double Apower = Apid + Aff;



        drive.update();
        flip.setPower(power);

        if(target == 0){
            flip.setPower(0);
        }


    }

    @Override
    public void stop() {
        //shuts down camera
//        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
//            visionPortal.stopLiveView();
//            visionPortal.stopStreaming();
//        }
//        // this closes down the portal when we stop the code
//        propProcessor.close();
//        visionPortal.close();
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