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
                                //insert april tag code before auto code
                                //read april tag
                                .forward(46)
                                //raise to tallest setting
                                //pivot arm to be near pole
                                //open grabber
                                //re-pivot
                                //go to ground arm setting
                                .back(12)
                                //if (tagOfInterest.id == LEFT_TAG_ID) {
                                //            // Drive to the Left Zone.
                                //            .forward(18)
                                //            .strafeLeft(18)
                                //        } else if (tagOfInterest.id == CENTER_TAG_ID) {
                                //            // Drive to the Center Zone.
                                //            stay
                                //        } else if (tagOfInterest.id == RIGHT_TAG_ID) {
                                //            // Drive to the Right Zone.
                                //            .forward(18)
                                //            .strafeRight(18)
                                //        }
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