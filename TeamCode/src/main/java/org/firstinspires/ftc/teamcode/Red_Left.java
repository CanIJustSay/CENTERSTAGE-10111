package org.firstinspires.ftc.teamcode;


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
public class Red_Left extends OpMode {
    private VisionPortal visionPortal;
    private PropDetectionPipeline propProcessor;
    private SampleMecanumDrive drive;
    private AprilTagProcessor atagProcessor;
    private PIDController controller;

    public static double p = 0.007, i = 0, d = 0.00;    //PID gains to be tuned

    public static double f = 0.16;
    public static int target = 1;    //target position for the arm
    public static double tick_in_degrees = 537.6 / 360;

    private DcMotorEx arm;
    private DcMotorEx flip;
    private DcMotorEx arm1;
    private Servo claw;
    private int targetTag; // what tag should we align with on the backboard - the ID
    private double detX;  // detected X values of the wanted tag
    private double detY; // detected Y values of the wanted tag

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

        // you may also want to take a look at some of the examples for instructions on
        // how to have a switchable camera (switch back and forth between two cameras)
        // or how to manually edit the exposure and gain, to account for different lighting conditions
        // these may be extra features for you to work on to ensure that your robot performs
        // consistently, even in different environments


        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        Pose2d startPose = new Pose2d(-34, -63.75, Math.toRadians(270.00));

        drive.setPoseEstimate(startPose);



                left =  drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{
                            claw.setPosition(0.85);
                        })
                        .setReversed(true)
                        .lineTo(new Vector2d(-46,-37))
                        .setReversed(false)
                        .splineTo(new Vector2d(-25,-60.5), Math.toRadians(0))
                        .waitSeconds(10)
                        .forward(60)
                        .addTemporalMarker(()->{
                            target = 220;
                            //raise arm
                            arm.setPower(0.35);
                            arm1.setPower(0.35);
                        })
                        .lineTo(new Vector2d(52.5,-33))
                        .addDisplacementMarker(145,()->{
                            //stop arm from going higher
                            arm.setPower(0.1);
                            arm1.setPower(0.1);
                        })
                        .addTemporalMarker(()->{
                            claw.setPosition(0.35);
                        })
                        .waitSeconds(2)
                        .back(5)
                        .build();


                traj = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{
                            claw.setPosition(0.85);
                        })
                        .back(30)
                        .setReversed(false)
                        .splineTo(new Vector2d(-25,-60.5), Math.toRadians(0))
                        .waitSeconds(10)
                        .forward(60)
                        .addTemporalMarker(()->{
                            target = 220;
                            //raise arm
                            arm.setPower(0.35);
                            arm1.setPower(0.35);
                        })
                        .lineTo(new Vector2d(52.5,-37))
                        .addDisplacementMarker(145,()->{
                            arm.setPower(0.1);
                            arm1.setPower(0.1);
                        })
                        .addTemporalMarker(()->{
                            claw.setPosition(0.35);
                        })
                        .waitSeconds(2)
                        .back(5)
                        .build();

                right =  drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{
                            claw.setPosition(0.85);
                        })
                        .back(10)
                        .setReversed(true)
                        .splineTo(new Vector2d(-28,-39), Math.toRadians(0))
                        .setReversed(false)
                        .splineTo(new Vector2d(-25,-60.5), Math.toRadians(0))
                        .waitSeconds(10)
                        .forward(60)
                        .addTemporalMarker(()->{
                            target = 220;
                            //raise arm
                            arm.setPower(0.35);
                            arm1.setPower(0.35);
                        })
                        .lineTo(new Vector2d(52.5,-48))
                        .addDisplacementMarker(135,()->{
                            arm.setPower(0.1);
                            arm1.setPower(0.1);
                        })
                        .addTemporalMarker(()->{
                            claw.setPosition(0.35);
                        })
                        .waitSeconds(2)
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

        PropDetectionPipeline.PropPositions recordedPropPosition = propProcessor.getRecordedPropPosition();

        if (propProcessor.getLargestContourArea() < 4000) {
            recordedPropPosition = RIGHT;
        }

        //right of the mat
        switch (recordedPropPosition) {

            case RIGHT:
                drive.followTrajectorySequenceAsync(right);
                break;

            case MIDDLE:
                drive.followTrajectorySequenceAsync(traj);
                break;
            case LEFT:
                drive.followTrajectorySequenceAsync(left);
                break;

        }
        propProcessor.close();
    }

    @Override
    public void loop() {
        // telemetryAprilTag();


        controller.setPID(p,i,d);
        int armPos = flip.getCurrentPosition();
        double pid = controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(target / tick_in_degrees)) * f;

        double power = pid + ff;

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
