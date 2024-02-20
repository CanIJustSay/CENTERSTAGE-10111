package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class Meep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(14.5, 63.75, Math.toRadians(90.00));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(startPose)
                                .addTemporalMarker(()->{

                                })
                                .back(29)
                                .setReversed(false)
                                .addTemporalMarker(()->{

                                })

                                .splineTo(new Vector2d(53,38.4),Math.toRadians(0))
                                .addTemporalMarker(()-> {
                                })
                                .waitSeconds(2)
                                .back(5)
                                .addTemporalMarker(()->{
                                })
                                .lineToLinearHeading(new Pose2d(34.3,9.7, Math.toRadians(180)))
                                .addDisplacementMarker(160,()->{
                                    //raise the arm and open up the claw
                                })
                                .addDisplacementMarker(166,()->{
                                   // arm1.setPower(0.1);
                                   // arm.setPower(0.1);
                                })
                                .lineTo(new Vector2d(-59,15.7))
                                .addTemporalMarker(()->{
                                 //   claw.setPosition(0.85);
                                })
                                .waitSeconds(1)
                                .lineTo(new Vector2d(12.3,6.7))
                                .addDisplacementMarker(230,()->{
                                  //  arm1.setPower(-0.1);
                                   // arm.setPower(-0.1);
                                })
                              //  .lineToLinearHeading(new Pose2d(60,11, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(38,6.7, Math.toRadians(0)))
                                .strafeLeft(55)
                                .forward(20)
                                .addTemporalMarker(()->{
                                   // claw.setPosition(0.35);
                                })
                                .back(5)
                                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}