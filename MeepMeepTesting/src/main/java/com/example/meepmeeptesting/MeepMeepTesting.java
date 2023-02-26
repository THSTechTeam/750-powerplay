package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(70, 35, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(3.0, 3.0), Math.toRadians(0.0))
                                .splineToConstantHeading(new Vector2d(45.0, 5.0), Math.toRadians(0.0))
                                .splineToSplineHeading(new Pose2d(57.0, -6.0, Math.toRadians(-60.0)), Math.toRadians(-40.0))
                                .waitSeconds(0.5)
                                .splineToSplineHeading(new Pose2d(55.0, 11.0, Math.toRadians(-90.0)), Math.toRadians(90.0))
                                .splineToConstantHeading(new Vector2d(54.0, 34.0), Math.toRadians(90.0))
                                .waitSeconds(0.5)
                                .splineToSplineHeading(new Pose2d(56.0, -3.5, Math.toRadians(-70.0)), Math.toRadians(-80.0))
                                .waitSeconds(0.5)
                                .splineToSplineHeading(new Pose2d(57.0, 11.0, Math.toRadians(-90.0)), Math.toRadians(90.0))
                                .splineToConstantHeading(new Vector2d(57.0, 35.0), Math.toRadians(90.0))
                                .waitSeconds(0.5)
                                .splineToSplineHeading(new Pose2d(58.0, -2.50, Math.toRadians(-70.0)), Math.toRadians(-80.0))
                                .waitSeconds(0.5)
                                .splineToSplineHeading(new Pose2d(57.0, 11.0, Math.toRadians(-90.0)), Math.toRadians(90.0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
/* close grabber, keep closed
.forward(30)
.forward(40)
.turn(Math.toRadians(90))
medium cone raise
open grabber
drop arm
.turn(Math.toRadians(90))
.back(10)
do apriltag code */

/*
.forward(30)
.turn(Math.toRadians(90))
.forward(30)
.turn(Math.toRadians(90))
.forward(30)
.turn(Math.toRadians(90))
.forward(30)
.turn(Math.toRadians(90))
.build()
 */