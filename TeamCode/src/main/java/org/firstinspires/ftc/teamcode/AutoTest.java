package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutoTest")
public class AutoTest extends OpMode {
    SampleMecanumDrive drive;
    @Override
    public void init(){
        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

        Pose2d startPose = new Pose2d(-8.08, 10.19, Math.toRadians(90.00));

        drive.setPoseEstimate(startPose);

        TrajectorySequence testTraj = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(12.64, 36.70), Math.toRadians(90.00))
                .build();

        drive.followTrajectorySequence(testTraj);

    }

    @Override
    public void loop(){

    }

    @Override
    public void stop(){

    }

}
