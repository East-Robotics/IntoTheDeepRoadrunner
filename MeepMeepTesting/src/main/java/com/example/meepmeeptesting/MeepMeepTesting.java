package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(720);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(10, -60, Math.toRadians(90)))
                        //Specimen 1
                        .forward(30)
                        .back(8)
                        .strafeRight(27)
                        //Push samples to zone
                        .forward(28)
                        .strafeRight(10)
                        .back(45)
                        .forward(45)
                        .strafeRight(10)
                        .back(45)
                        .forward(45)
                        .strafeRight(5)
                        .back(45)
                        //2nd Specimen
                        .strafeTo(new Vector2d(38,-60))
                        .strafeTo(new Vector2d(0,-35))
                        //3rd Specimen
                        .strafeTo(new Vector2d(38,-60))
                        .strafeTo(new Vector2d(0,-35))
                        //4th Specimen
                        .strafeTo(new Vector2d(38,-60))
                        .strafeTo(new Vector2d(0,-35))
                        //5th Speciemen
                        .strafeTo(new Vector2d(38,-60))
                        .strafeTo(new Vector2d(0,-35))

                        //.line
                        .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

/* 5+0
.followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(10, -60, Math.toRadians(90)))
                        .forward(30)
                        .back(8)
                        .strafeRight(27)
                        .forward(28)
                        .strafeRight(10)
                        .back(45)
                        .forward(45)
                        .strafeRight(10)
                        .back(45)
                        .forward(45)
                        .strafeRight(5)
                        .back(45)
                        .splineToConstantHeading(new Vector2d(58,-60), Math.toRadians(0))
                        .turn(Math.toRadians(-90))
                        .forward(3)
                        .splineToConstantHeading(new Vector2d(0,-45), -90)
                        .turn(Math.toRadians(90))
                        .forward(15)

 */
/*
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-10, -60, Math.toRadians(90)))
                        .forward(30)
                        .back(8)
                        .strafeLeft(37)
                        .strafeTo(new Vector2d(-55,-55))
                        .turn(Math.toRadians(-45))
                        .turn(Math.toRadians(47))
                        .turn(Math.toRadians(-45))
                        .turn(Math.toRadians(65))
                        .strafeTo(new Vector2d(-60,-45))
                        .strafeTo(new Vector2d(-55,-55))
                        .turn(Math.toRadians(-65))
                        .strafeTo(new Vector2d(-25,-5))
                        .turn(Math.toRadians(-45))
                        .waitSeconds(2)
                        .strafeTo(new Vector2d(-55,-55))
                        .turn(Math.toRadians(45))
                        .strafeTo(new Vector2d(-25,-5))
                        .turn(Math.toRadians(-45))
 */