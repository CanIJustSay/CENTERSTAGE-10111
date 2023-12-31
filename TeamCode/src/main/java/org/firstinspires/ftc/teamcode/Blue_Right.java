package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.CameraCode.PropDetectionPipeline.PropPositions.MIDDLE;
import static org.firstinspires.ftc.teamcode.CameraCode.PropDetectionPipeline.PropPositions.UNFOUND;

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
public class Blue_Right extends OpMode {
    private VisionPortal visionPortal;
    private PropDetectionPipeline propProcessor;
    private SampleMecanumDrive drive;
    private AprilTagProcessor atagProcessor;
    private PIDController controller;

    public static double p = 0.056, i = 0, d = 0.001;    //PID gains to be tuned

    public static double f = 0.01;
    public static int target = 0;    //target position for the arm
    public static double tick_in_degrees = 537.6 / 360;
    private DcMotor intake;
    private DcMotorEx arm;
    private Servo wrist;
    private Servo knuckle;
    private int targetTag; // what tag should we align with on the backboard - the ID
    private double detX;  // detected X values of the wanted tag
    private double detY; // detected Y values of the wanted tag


    @Override
    public void init() {

        // the current range set by lower and upper is the full range
        // HSV takes the form: (HUE, SATURATION, VALUE)
        // which means to select our colour, only need to change HUE
        // the domains are: ([0, 180], [0, 255], [0, 255])
        // this is tuned to detect red, so you will need to experiment to fine tune it for your robot
        // and experiment to fine tune it for blue

        Scalar lower = new Scalar(97,100,50);
        Scalar upper = new Scalar(125,255,255);

        double minArea = 200; // the minimum area for the detection to consider for your prop
        drive = new SampleMecanumDrive(hardwareMap);

        controller = new PIDController(p,i,d);
        atagProcessor = new AprilTagProcessor.Builder().build();

        arm = hardwareMap.get(DcMotorEx.class,"arm");

        wrist = hardwareMap.get(Servo.class,"wrist");

        knuckle = hardwareMap.get(Servo.class,"knuckle");


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
                .build();

        // you may also want to take a look at some of the examples for instructions on
        // how to have a switchable camera (switch back and forth between two cameras)
        // or how to manually edit the exposure and gain, to account for different lighting conditions
        // these may be extra features for you to work on to ensure that your robot performs
        // consistently, even in different environments
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", propProcessor.getRecordedPropPosition());
        //telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + propProcessor.getLargestContourX() + ", y: " + propProcessor.getLargestContourY());
        //telemetry.addData("Currently Detected Mass Area", propProcessor.getLargestContourArea());
        telemetryAprilTag();

    }

    @Override
    public void start() {
        propProcessor.close();

/*
        TODO
              check up on rr - could start being dumb again, tuning wise
              widen left and right bounds because middle needs to be larger
 */
        PropDetectionPipeline.PropPositions recordedPropPosition = propProcessor.getRecordedPropPosition();

        if (recordedPropPosition == UNFOUND) {
            // this is a guess. doubtful it'll be needed but you never know
            recordedPropPosition = MIDDLE;
        }
        wrist.setPosition(0.57);
        knuckle.setPosition(0.35);
        //right of mat
        Pose2d startPose = new Pose2d(-33.89, 63.75, Math.toRadians(270.00));

        switch (recordedPropPosition) {
            case LEFT:

                drive.setPoseEstimate(startPose);

                TrajectorySequence left =  drive.trajectorySequenceBuilder(startPose)
                        .splineTo(new Vector2d(-28.89, 38.0), Math.toRadians(-45))
                        .setReversed(true)
                        .splineTo( new Vector2d(-27.0, 58.5), Math.toRadians(0.0))

                        //  .lineTo(new Vector2d(0,36))
                        .addTemporalMarker(()->{
                            //does something


                        })
                        .build();

                drive.followTrajectorySequence(left);

                break;

            case MIDDLE:

                drive.setPoseEstimate(startPose);

                TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(new Vector2d(-33.89, 32.0))
                        .setReversed(true)
                        .back(20)
                        .splineTo( new Vector2d(-21.0, 58.5), Math.toRadians(0.0))

                        // .lineTo(new Vector2d(0,36))
                        .addTemporalMarker(()->{
                            //does something


                        })
                        .build();

                drive.followTrajectorySequence(middle);

                break;

            case RIGHT:


                drive.setPoseEstimate(startPose);

                TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                        .splineTo(new Vector2d(-42.5, 42.0), Math.toRadians(225.0))
                        .setReversed(true)
                        .splineTo( new Vector2d(-21.0, 58.5), Math.toRadians(0.0))
                        // .lineTo(new Vector2d(0,36))
                        .addTemporalMarker(()->{
                            //does something

                        })
                        .build();

                drive.followTrajectorySequence(right);
                break;
        }

    }

    @Override
    public void loop() {
        // telemetryAprilTag();


        // for rr heading has some error - may be a problem.
        // use "yaw" ONLY after x value is close to 0 relative to the april tag.
        // "yaw" should fix any heading error after rr path
        // ^thought process only good for backboard april tags.

        /*
         * TODO
         *  tune pid gains for arm
         */
        controller.setPID(p,i,d);
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(target / tick_in_degrees)) * f;

        double power = pid + ff;

        arm.setPower(power);

    }

    @Override
    public void stop() {

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