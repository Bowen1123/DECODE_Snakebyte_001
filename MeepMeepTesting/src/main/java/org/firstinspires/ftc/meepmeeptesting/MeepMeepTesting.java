package org.firstinspires.ftc.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
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

        RoadRunnerBotEntity league2Bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 40, Math.PI, Math.PI, 15)
                .setDimensions(15,15)
                .build();
        RoadRunnerBotEntity league3Bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 45, Math.PI/2, Math.PI/2, 15)
                .setDimensions(15,15)
                .build();

        //blue_goal(myBot);
        //blue_goal_new_gate(league2Bot);
        //blue_goal_new_nogate(league2Bot);
        //blue_far(league2Bot);

        gate_blue_goal_league3(league3Bot);

        /// blue_goal_league2(league2Bot);
        /// red_goal_league2(league2Bot);
        /// blue_far_league2_far_spike(league2Bot);
        /// red_far_league2_far_spike(league2Bot);
        /// blue_far_league2_middle_spike(league2Bot);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(league3Bot)
                .start();
    }

    ///  League 3 ///

    static void blue_far_league3(RoadRunnerBotEntity myBot){
        Pose2d initialPose = new Pose2d(60,-12,Math.toRadians(200));

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .setTangent(180)
                .splineToLinearHeading(new Pose2d(60, -40, Math.toRadians(270)), Math.toRadians(270))
                .build());
    }

    static void gate_blue_goal_league3(RoadRunnerBotEntity myBot){

        Pose2d initialPose = new Pose2d(-52,-52,Math.toRadians(225));

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .lineToX(-12)

                .setTangent(Math.toRadians(315)) // leave goal
                .splineToLinearHeading(new Pose2d(12, -30, Math.toRadians(270)), Math.toRadians(270))

                .setTangent(Math.toRadians(270)) // get spike
                .lineToY(-52, new TranslationalVelConstraint(16))

                .setTangent(Math.toRadians(180)) // to gate
                .splineToConstantHeading(new Vector2d(0, -55), Math.toRadians(180))

                .setTangent(Math.toRadians(90)) // to shoot
                .splineToLinearHeading(new Pose2d(-12,-12,Math.toRadians(225)), Math.toRadians(135))

                ///
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(-12, -30, Math.toRadians(270)), Math.toRadians(270))

                .setTangent(Math.toRadians(270))
                .lineToY(-52, new TranslationalVelConstraint(16))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-12,-12,Math.toRadians(225)), Math.toRadians(135))

                ///
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(36, -30, Math.toRadians(270)), Math.toRadians(270))

                .setTangent(Math.toRadians(270))
                .lineToY(-52, new TranslationalVelConstraint(16))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-12,-12,Math.toRadians(225)), Math.toRadians(135))


                .setTangent(Math.toRadians(180))
                .lineToX(-60)

                .build());
    }
    static void basic_blue_goal_league3(RoadRunnerBotEntity myBot){

        Pose2d initialPose = new Pose2d(-52,-52,Math.toRadians(225));

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .lineToX(-12)


                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(-12, -30, Math.toRadians(270)), Math.toRadians(270))

                .setTangent(Math.toRadians(270))
                .lineToY(-52, new TranslationalVelConstraint(16))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-12,-12,Math.toRadians(225)), Math.toRadians(90))

                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(12, -30, Math.toRadians(270)), Math.toRadians(270))

                .setTangent(Math.toRadians(270))
                .lineToY(-52, new TranslationalVelConstraint(16))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-12,-12,Math.toRadians(225)), Math.toRadians(135))

                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(36, -30, Math.toRadians(270)), Math.toRadians(270))

                .setTangent(Math.toRadians(270))
                .lineToY(-52, new TranslationalVelConstraint(16))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-12,-12,Math.toRadians(225)), Math.toRadians(135))

                .setTangent(Math.toRadians(180))
                .lineToX(-60)

                .build());
    }




    ///  League 2 ///
    static void red_goal_league2 (RoadRunnerBotEntity myBot){

        Pose2d initialPose = new Pose2d(-52,52,Math.toRadians(-225));

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .lineToX(-18)


                .setTangent(Math.toRadians(-315))
                .splineToLinearHeading(new Pose2d(-12, 30, Math.toRadians(-270)), Math.toRadians(-270))

                .setTangent(Math.toRadians(-270))
                .lineToY(52, new TranslationalVelConstraint(16))

                .setTangent(-90)
                .splineToLinearHeading(new Pose2d(-18,18,Math.toRadians(-225)), Math.toRadians(-90))

                .setTangent(Math.toRadians(-315))
                .splineToLinearHeading(new Pose2d(12, 30, Math.toRadians(-270)), Math.toRadians(-270))

                .setTangent(Math.toRadians(-270))
                .lineToY(52, new TranslationalVelConstraint(16))

                .setTangent(-90)
                .splineToLinearHeading(new Pose2d(-18,18,Math.toRadians(-225)), Math.toRadians(-135))

                .setTangent(Math.toRadians(180))
                .lineToX(-60)

                .build());
    }
    static void blue_goal_league2 (RoadRunnerBotEntity myBot){

        Pose2d initialPose = new Pose2d(-52,-52,Math.toRadians(225));

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .lineToX(-18)


                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(-12, -30, Math.toRadians(270)), Math.toRadians(270))

                .setTangent(Math.toRadians(270))
                .lineToY(-52, new TranslationalVelConstraint(16))

                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-18,-18,Math.toRadians(225)), Math.toRadians(90))

                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(12, -30, Math.toRadians(270)), Math.toRadians(270))

                .setTangent(Math.toRadians(270))
                .lineToY(-52, new TranslationalVelConstraint(16))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-18,-18,Math.toRadians(225)), Math.toRadians(135))

                .setTangent(Math.toRadians(180))
                .lineToX(-60)

                .build());
    }

    static void blue_far_league2_far_spike(RoadRunnerBotEntity myBot){

        Pose2d initialPose = new Pose2d(60,-10,Math.toRadians(180));

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-18,-18, Math.toRadians(225)), Math.toRadians(180))

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(36, -30, Math.toRadians(270)), Math.toRadians(270))

                .setTangent(Math.toRadians(270))
                .lineToY(-52, new TranslationalVelConstraint(16))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-18,-18, Math.toRadians(225)), Math.toRadians(180))


                .lineToX(-60)
                .build());
    }

    static void red_far_league2_far_spike(RoadRunnerBotEntity myBot){

        Pose2d initialPose = new Pose2d(60,10,Math.toRadians(-180));

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-18,18, Math.toRadians(-225)), Math.toRadians(-180))

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(36, 30, Math.toRadians(-270)), Math.toRadians(-270))

                .setTangent(Math.toRadians(-270))
                .lineToY(52, new TranslationalVelConstraint(16))

                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-18,18, Math.toRadians(-225)), Math.toRadians(-180))


                .lineToX(-60)
                .build());
    }

    static void blue_far_league2_middle_spike(RoadRunnerBotEntity myBot){

        Pose2d initialPose = new Pose2d(60,-10,Math.toRadians(180));

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-18,-18, Math.toRadians(225)), Math.toRadians(180))

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(12, -30, Math.toRadians(270)), Math.toRadians(270))

                .setTangent(Math.toRadians(270))
                .lineToY(-52, new TranslationalVelConstraint(16))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-18,-18, Math.toRadians(225)), Math.toRadians(180))

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-12, -30, Math.toRadians(270)), Math.toRadians(270))

                .setTangent(Math.toRadians(270))
                .lineToY(-52, new TranslationalVelConstraint(16))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-18,-18, Math.toRadians(225)), Math.toRadians(180))

                .lineToX(-60)
                .build());
    }

    static void red_far_league2_middle_spike(RoadRunnerBotEntity myBot){

        Pose2d initialPose = new Pose2d(60,10,Math.toRadians(-180));

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-18,18, Math.toRadians(-225)), Math.toRadians(-180))

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(12, 30, Math.toRadians(-270)), Math.toRadians(-270))

                .setTangent(Math.toRadians(-270))
                .lineToY(52, new TranslationalVelConstraint(16))

                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-18,18, Math.toRadians(-225)), Math.toRadians(-180))


                .lineToX(-60)
                .build());
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

    ///  League 1 ///
    static void blue_far(RoadRunnerBotEntity myBot){
        Pose2d initialPose = new Pose2d(64,-10,Math.toRadians(180));

        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -34) {
                return 20.0;
            } else {
                return 50.0;
            }
        };

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)

                .splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(225)), Math.toRadians(180))

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-12, -52, Math.toRadians(270)), Math.toRadians(270))

                .setTangent(45)
                        .splineToSplineHeading(new Pose2d(-12, -12, Math.toRadians(225)), Math.toRadians(135))

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