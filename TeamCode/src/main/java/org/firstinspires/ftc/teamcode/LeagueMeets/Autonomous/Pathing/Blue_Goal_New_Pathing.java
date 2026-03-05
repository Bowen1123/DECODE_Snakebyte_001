package org.firstinspires.ftc.teamcode.LeagueMeets.Autonomous.Pathing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LeagueMeets.Mechanism_League2;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
@Disabled

public class Blue_Goal_New_Pathing extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-52,-52,Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Mechanism_League2 mechanism = new Mechanism_League2(hardwareMap);


        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -30.0) {
                return 20.0;
            } else {
                return 50.0;
            }
        };

        TrajectoryActionBuilder defaultPath = drive.actionBuilder(initialPose)
                ///  START & CYCLE
                .lineToX(-24)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-12,-52, Math.toRadians(270)), Math.toRadians(265), baseVelConstraint)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-24,-24, Math.toRadians(225)), Math.toRadians(90))

                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(12,-52, Math.toRadians(270)), Math.toRadians(265), new TranslationalVelConstraint(30))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-24,-24, Math.toRadians(225)), Math.toRadians(180))

                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(36,-52, Math.toRadians(270)), Math.toRadians(265), new TranslationalVelConstraint(30))
                .setTangent(Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(-24,-24, Math.toRadians(225)), Math.toRadians(180))

                .setTangent(Math.toRadians(180))
                .lineToX(-60);

        waitForStart();

        Actions.runBlocking(defaultPath.build());


    }
}