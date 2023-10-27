package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.CameraCode.PropDetectionPipeline.PropPositions.MIDDLE;
import static org.firstinspires.ftc.teamcode.CameraCode.PropDetectionPipeline.PropPositions.UNFOUND;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
public class Right_Close_Auto extends OpMode {
    private VisionPortal visionPortal;
    private PropDetectionPipeline propProcessor;
    private SampleMecanumDrive drive;
    private AprilTagProcessor atagProcessor;
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;    //PID gains to be tuned
    public static double f = 0;
    public static int target = 0;    //target position for the arm
    public static double tick_in_degrees = 537.6 / 360;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor intake;
    private DcMotorEx arm;

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
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

    //    arm = hardwareMap.get(DcMotorEx.class,"arm"); // not yet on bot - may throw errors

        // values are for blue
        // not consistent at all.
        //Scalar lower = new Scalar(97,100,100);
        //Scalar upper = new Scalar(125,255,255);

        // values are for red
        // very consistent
        Scalar lower = new Scalar(150, 100, 100);
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
    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", propProcessor.getRecordedPropPosition());
        telemetry.addData("Currently Detected Mass Center", "x: " + propProcessor.getLargestContourX() + ", y: " + propProcessor.getLargestContourY());
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

        if (recordedPropPosition == UNFOUND) {
            // this is a guess. doubtful it'll be needed but you never know
            recordedPropPosition = MIDDLE;
        }

        //right side of mat(allegedly)
        Pose2d startPose = new Pose2d(14.5, -63.75, Math.toRadians(90.00));
        switch (recordedPropPosition) {
            case LEFT:

                drive.setPoseEstimate(startPose);

                TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                        .splineTo(new Vector2d(6.85,-36.04), Math.toRadians(125.00))
                        .setReversed(true)
                        .splineTo(new Vector2d(48.93,-27.22),Math.toRadians(0))
                        .build();
                drive.followTrajectorySequence(left);
                break;

            case MIDDLE:

                drive.setPoseEstimate(startPose);

               TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                        .splineTo(new Vector2d(14.50, -32), Math.toRadians(90.00))
                        .setReversed(true)
                        .splineTo(new Vector2d(46.54, -37.23), Math.toRadians(0.00))
                        .build();


                drive.followTrajectorySequence(traj);

                break;

            case RIGHT:

                drive.setPoseEstimate(startPose);

                TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                        .splineTo(new Vector2d(23.65, -41.27), Math.toRadians(90))
                        .setReversed(true)
                        .splineTo(new Vector2d(49.63,-43.73),Math.toRadians(0))
                        .build();

                drive.followTrajectorySequence(right);
                break;
        }

    }

    @Override
    public void loop() {
        telemetryAprilTag();
        propProcessor.close();


        // for rr heading has some error - may be a problem.
        // use "yaw" ONLY after x value is close to 0 relative to the april tag.
        // "yaw" should fix any heading error after rr path
        // ^thought process only good for backboard april tags.

        // this works -
        // drives until "Y" is less than 50. use for rr maybe.
        /**
 List<AprilTagDetection> allDets = atagProcessor.getDetections();
 for (AprilTagDetection det : allDets) {

 // targetTag isolated
 if (det.id == targetTag) {
 detX = det.ftcPose.x;
 detY = det.ftcPose.y;


 if (detY > 50) {
 rightFront.setPower(0.1);
 rightBack.setPower(0.1);
 leftBack.setPower(0.1);
 leftFront.setPower(0.1);
 } else {

 rightFront.setPower(0);
 rightBack.setPower(0);
 leftBack.setPower(0);
 leftFront.setPower(0);
 }



 }
 }
 */

        /*
         * TODO
         *  tune pid gains for arm
         */

//        controller.setPID(p,i,d);
//        int armPos = arm.getCurrentPosition();
//        double pid = controller.calculate(armPos,target);
//        double ff = Math.cos(Math.toRadians(target / tick_in_degrees)) * f;
//
//        double power = pid + ff;
//        arm.setPower(power);

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