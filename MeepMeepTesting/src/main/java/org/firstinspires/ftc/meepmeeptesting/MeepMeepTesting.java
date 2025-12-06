package org.firstinspires.ftc.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(44, 36, Math.PI, Math.PI, 15)
                .setDimensions(15,15)
                .build();

        RoadRunnerBotEntity newBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 40, Math.PI/2, Math.PI/2, 15)
                .setDimensions(15,15)
                .build();

        //blue_goal(myBot);
        red_goal_new_nogate(newBot);
        //blue_goal_new_nogate(newBot);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(newBot)
                .start();
    }

    static void red_goal_new_nogate(RoadRunnerBotEntity myBot){

        Pose2d initialPose = new Pose2d(-52,52,Math.toRadians(-225));

        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() > 34) {
                return 20.0;
            } else {
                return 50.0;
            }
        };

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .lineToX(-24)


                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-12,52, Math.toRadians(-270)), Math.toRadians(-265), baseVelConstraint)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-24,24, Math.toRadians(-225)), Math.toRadians(-90))

                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(12,52, Math.toRadians(-270)), Math.toRadians(-265), baseVelConstraint)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-24,24, Math.toRadians(-225)), Math.toRadians(-180))

                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(36,52, Math.toRadians(-270)), Math.toRadians(-265), baseVelConstraint)
                .setTangent(Math.toRadians(-135))
                .splineToSplineHeading(new Pose2d(-24,24, Math.toRadians(-225)), Math.toRadians(-180))

                .setTangent(Math.toRadians(-180))
                .lineToX(-60)

                .build());
    }
    static void red_goal_new_gate(RoadRunnerBotEntity myBot){

        Pose2d initialPose = new Pose2d(-52,52,Math.toRadians(-225));

        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() > 34) {
                return 20.0;
            } else {
                return 50.0;
            }
        };

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .lineToX(-24)


                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-12,52, Math.toRadians(-270)), Math.toRadians(-265), baseVelConstraint)


                /// GATE, NOT ADDED TO AUTOs ///
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(0, 56, Math.toRadians(-180)), Math.toRadians(-270))
                // .lineToY(-50)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-24,24, Math.toRadians(-225)), Math.toRadians(-90))

                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-24,24, Math.toRadians(-225)), Math.toRadians(-90))

                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(12,52, Math.toRadians(-270)), Math.toRadians(-265), baseVelConstraint)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-24,24, Math.toRadians(-225)), Math.toRadians(-180))

                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(36,52, Math.toRadians(-270)), Math.toRadians(-265), baseVelConstraint)
                .setTangent(Math.toRadians(-135))
                .splineToSplineHeading(new Pose2d(-24,24, Math.toRadians(-225)), Math.toRadians(-180))

                .setTangent(Math.toRadians(-180))
                .lineToX(-60)

                .build());
    }
    static void blue_goal_new_nogate(RoadRunnerBotEntity myBot){

        Pose2d initialPose = new Pose2d(-52,-52,Math.toRadians(225));

        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -34) {
                return 20.0;
            } else {
                return 50.0;
            }
        };

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .lineToX(-24)


                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-12,-52, Math.toRadians(270)), Math.toRadians(265), baseVelConstraint)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-24,-24, Math.toRadians(225)), Math.toRadians(90))

                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(12,-52, Math.toRadians(270)), Math.toRadians(265), baseVelConstraint)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-24,-24, Math.toRadians(225)), Math.toRadians(180))

                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(36,-52, Math.toRadians(270)), Math.toRadians(265), baseVelConstraint)
                .setTangent(Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(-24,-24, Math.toRadians(225)), Math.toRadians(180))

                .setTangent(Math.toRadians(180))
                .lineToX(-60)

                .build());
    }
    static void blue_goal_new_gate(RoadRunnerBotEntity myBot){

        Pose2d initialPose = new Pose2d(-52,-52,Math.toRadians(225));

        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -34) {
                return 20.0;
            } else {
                return 50.0;
            }
        };

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .lineToX(-24)


                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-12,-52, Math.toRadians(270)), Math.toRadians(265), baseVelConstraint)


                        /// GATE, NOT ADDED TO AUTOs ///
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(0, -56, Math.toRadians(180)), Math.toRadians(270))
                        // .lineToY(-50)
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(-24,-24, Math.toRadians(225)), Math.toRadians(90))

                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-24,-24, Math.toRadians(225)), Math.toRadians(90))

                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(12,-52, Math.toRadians(270)), Math.toRadians(265), baseVelConstraint)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-24,-24, Math.toRadians(225)), Math.toRadians(180))

                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(36,-52, Math.toRadians(270)), Math.toRadians(265), baseVelConstraint)
                .setTangent(Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(-24,-24, Math.toRadians(225)), Math.toRadians(180))

                        .setTangent(Math.toRadians(180))
                .lineToX(-60)

                .build());
    }

    static void blue_goal_league1(RoadRunnerBotEntity myBot){

        Pose2d initialPose = new Pose2d(-52,-52,Math.toRadians(225));

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .lineToX(-24)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-12, -28, Math.toRadians(270)), Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .lineToY(-52, new TranslationalVelConstraint(14))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-24,-24, Math.toRadians(225)), Math.toRadians(180), new TranslationalVelConstraint(44))

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(12, -28, Math.toRadians(270)), Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .lineToY(-52, new TranslationalVelConstraint(14))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-24,-24, Math.toRadians(225)), Math.toRadians(180), new TranslationalVelConstraint(44))

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(36, -28, Math.toRadians(270)), Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .lineToY(-52, new TranslationalVelConstraint(14))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-24,-24, Math.toRadians(225)), Math.toRadians(180), new TranslationalVelConstraint(44))

                .lineToX(-60)

                .build());
    }
    static void red_goal_league1(RoadRunnerBotEntity myBot){

        Pose2d initialPose = new Pose2d(-52,52,Math.toRadians(135));

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .lineToX(-24)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(-12, 28, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .lineToY(52, new TranslationalVelConstraint(14))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-24, 24, Math.toRadians(135)), Math.toRadians(180), new TranslationalVelConstraint(44) )

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(12, 28, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .lineToY(52, new TranslationalVelConstraint(14))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-24,24, Math.toRadians(135)), Math.toRadians(180), new TranslationalVelConstraint(44))

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(36, 28, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .lineToY(52, new TranslationalVelConstraint(14))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-24,24, Math.toRadians(135)), Math.toRadians(180), new TranslationalVelConstraint(44))

                .lineToX(-60)

                .build());
    }

}