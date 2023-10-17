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
        Pose2d startPose = new Pose2d(-63.57, -35.82, Math.toRadians(0.00));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)


                .setConstraints(49, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
// RFA TESTED
// LFA TESTED
// LCA TESTED
// RCA TESTED

                        drive.trajectorySequenceBuilder(startPose)

                                .splineTo(new Vector2d(-33.01, 22.65), Math.toRadians(90.00))
                                .splineTo(new Vector2d(-35.65, 49.00), Math.toRadians(90.00))
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}