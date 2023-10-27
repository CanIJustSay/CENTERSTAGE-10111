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



    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

        Pose2d startPose = new Pose2d(14.5, -63.75, Math.toRadians(90.00));

        drive.setPoseEstimate(startPose);

//                TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(14.5, -63.75, Math.toRadians(90.00)))
//                       // .lineTo(new Vector2d(14.5, -23.59))
//                        .forward(30)
////                        .splineTo(new Vector2d(0, -23.59),Math.toRadians(90))
////                        .waitSeconds(2.0)
////                        .setReversed(true)
////                        .splineTo(new Vector2d(37, -20), Math.toRadians(0.00))
//                        .build();
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(14.5,-23.75))
                .build();

        drive.followTrajectorySequence(traj);

    }

    @Override
    public void loop(){


    }

    @Override
    public void stop(){

    }

}
