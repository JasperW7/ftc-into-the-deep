package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

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

                        .splineToConstantHeading(new Vector2d(-10,-38),0) //to high rung

                        .strafeLeft(38) //to first sample
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-50,-50, Math.toRadians(45)), Math.toRadians(270))// high basket 1
                        .setReversed(false)

                        .splineToLinearHeading(new Pose2d(-58,-38,Math.toRadians(90)),Math.toRadians(90)) //to second sample
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-50,-50, Math.toRadians(45)), Math.toRadians(270))// high basket 2
                        .setReversed(false)

                        .splineToLinearHeading(new Pose2d(-55,-25,Math.toRadians(180)),Math.toRadians(180)) // to third sample
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-50,-50, Math.toRadians(45)), Math.toRadians(270))// high basket 3
                        .setReversed(false)

                        .splineToLinearHeading(new Pose2d(-26,-10,Math.toRadians(0)),Math.toRadians(0)) //park
















                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}