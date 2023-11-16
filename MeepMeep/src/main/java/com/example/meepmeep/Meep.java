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
        Pose2d startPose = new Pose2d(-33.89, 63.75, Math.toRadians(270.00));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(startPose)
                                .splineTo(new Vector2d(-28.89, 38.0), Math.toRadians(-45))
                                .setReversed(true)
                                .splineTo( new Vector2d(-27.0, 58.5), Math.toRadians(0.0))
                                .back(62)
                                .lineTo(new Vector2d(48,34))
                                //  .lineTo(new Vector2d(0,36))
                                .addTemporalMarker(()->{
                                    //does something
                                   // target = 100;
                                   // wrist.setPosition(1);

                                })
                                .lineTo(new Vector2d(48,50))
                                .build());





        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}