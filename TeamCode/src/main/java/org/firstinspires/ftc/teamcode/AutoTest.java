package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.CameraCode.WhitePixelDetectionPipeline.getDistance;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CameraCode.WhitePixelDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "AutoTest")
public class AutoTest extends OpMode {
    SampleMecanumDrive drive;
    VisionPortal visionPortal;
    WhitePixelDetectionPipeline pixelDetectionPipeline;


    @Override
    public void init(){
        drive = new SampleMecanumDrive(hardwareMap);

        pixelDetectionPipeline = new WhitePixelDetectionPipeline();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "logi"))
                .addProcessor(pixelDetectionPipeline)
                .build();


    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

        Pose2d startPose = new Pose2d(-8.08, 10.19, Math.toRadians(90.00));

        drive.setPoseEstimate(startPose);

//        TrajectorySequence testTraj = drive.trajectorySequenceBuilder(startPose)
//                .splineTo(new Vector2d(12.64, 36.70), Math.toRadians(90.00))
//                .build();
//
//        drive.followTrajectorySequence(testTraj);

    }

    @Override
    public void loop(){
        telemetry.addData("Coordinate", "(" + (int) pixelDetectionPipeline.cX + ", " + (int) pixelDetectionPipeline.cY + ")");
        telemetry.addData("Distance in Inch", getDistance(pixelDetectionPipeline.width));
        telemetry.update();


    }

    @Override
    public void stop(){

    }

}
