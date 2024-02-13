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

        Pose2d startPose = new Pose2d(-34, 63.75, Math.toRadians(90.00));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(startPose)
                                .addTemporalMarker(()->{
                                  //  claw.setPosition(0.85);
                                })
                                .back(10)
                                .setReversed(true)
                                .splineTo(new Vector2d(-28,39), Math.toRadians(0))
                                .setReversed(false)
                                .splineTo(new Vector2d(-25,60.5), Math.toRadians(0))
                                .waitSeconds(10)
                                .forward(60)
                                .addTemporalMarker(()->{
                                   // target = 220;
                                    //raise arm
                                  //  arm.setPower(0.35);
                                   // arm1.setPower(0.35);
                                })
                                .lineTo(new Vector2d(52.5,40))
                                .addDisplacementMarker(135,()->{
                                  //  arm.setPower(0.1);
                                  //  arm1.setPower(0.1);
                                })
                                .addTemporalMarker(()->{
                                   // claw.setPosition(0.35);
                                })
                                .waitSeconds(2)
                                .back(5)
                                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}