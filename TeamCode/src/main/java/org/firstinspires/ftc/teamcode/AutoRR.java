package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.roadrunner.mecanum.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "LineTwiceAuto", group = "Autonomous")
public class AutoRR extends LinearOpMode {

    private SampleMecanumDrive drive;

/**
 * AprilTag relocal - basically using the setPoseEstimate.
 *
 *
 */

        @Override
        public void runOpMode() {
            // Initialize the drive system
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            waitForStart();
            // We want to start the bot at x: 10, y: -8, heading: 90 degrees
            Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

            drive.setPoseEstimate(startPose);

            Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(30, 30), 0)
                    .build();
//            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                    .splineTo(new Vector2d(20, 9), Math.toRadians(45))
//                    .build();

            drive.followTrajectory(traj);
           // drive.followTrajectory(traj2);

        }
}