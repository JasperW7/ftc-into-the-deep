package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-34, -60, Math.toRadians(90)))
//                        .forward(10) //high rung
//                        .strafeRight(30)
//                        .forward(15)
//
//                        .back(15) //sample 1
//                        .strafeLeft(44)
//                        .forward(12) //grab
//
//                        .back(15)
//                        .turn(Math.toRadians(-45))
//                        .strafeLeft(3)//high basket
//                        .strafeRight (3)
//
//                        .turn(Math.toRadians(45)) //sample 2
//                        .forward(15)
//                        .strafeLeft(10) //grab
//                        .strafeRight(3)
//
//                        .back(10)
//                        .turn(Math.toRadians(-45))
//                        .strafeRight(2)//high basket

                        .splineTo(new Vector2d(-10,-20),Math.toRadians(90))







                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}