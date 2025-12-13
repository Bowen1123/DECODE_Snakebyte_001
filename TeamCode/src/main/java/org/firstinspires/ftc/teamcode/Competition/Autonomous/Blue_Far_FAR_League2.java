package org.firstinspires.ftc.teamcode.Competition.Autonomous;


import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.Mechanism;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class Blue_Far_FAR_League2 extends LinearOpMode {
    private double initX = 60, initY = -10, initHeading = Math.toRadians(180);
    private double goalX = -18, goalY = -18, goalHeading = Math.toRadians(225);
    private double spikeX = -12 /*12*/ , spikeY = -30, spikeHeading = Math.toRadians(270);
    private double intakeSpikeY = -52;

    private TrajectoryActionBuilder start, spike1, spike2, spike3, gate, goal1, goal2, goal3, leave;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(initX, initY, initHeading);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Mechanism mechanism = new Mechanism(hardwareMap);

        double spike_x_gap = 24;

        TranslationalVelConstraint slow = new TranslationalVelConstraint(14);

        Pose2d goalPose = new Pose2d(goalX, goalY, goalHeading);
        Pose2d gatePose = new Pose2d(0, -56, Math.toRadians(180));
        Pose2d spikeOnePose = new Pose2d(spikeX, spikeY, spikeHeading);
        Pose2d spikeTwoPose = new Pose2d(spikeX + spike_x_gap, spikeY, spikeHeading);
        Pose2d spikeThreePose = new Pose2d(spikeX + (2 * spike_x_gap), spikeY, spikeHeading);

        Pose2d spikeOneIntookPose = new Pose2d(spikeOnePose.position.x, intakeSpikeY, spikeOnePose.heading.toDouble());
        Pose2d spikeTwoIntookPose = new Pose2d(spikeTwoPose.position.x, intakeSpikeY, spikeTwoPose.heading.toDouble());
        Pose2d spikeThreeIntookPose = new Pose2d(spikeThreePose.position.x, intakeSpikeY, spikeThreePose.heading.toDouble());



        TrajectoryActionBuilder start = drive.actionBuilder(initialPose)
                .splineToLinearHeading(goalPose, Math.toRadians(180));

        TrajectoryActionBuilder spikeThree = drive.actionBuilder(goalPose)
                .setTangent(0)
                .splineToLinearHeading(spikeThreePose, spikeHeading);

        TrajectoryActionBuilder spikeThreeIntake = drive.actionBuilder(goalPose)
                .lineToY(spikeY, new TranslationalVelConstraint(16));

        TrajectoryActionBuilder goal3 = drive.actionBuilder(spikeThreeIntookPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(goalPose, Math.toRadians(180));

        TrajectoryActionBuilder end = drive.actionBuilder(goalPose)
                .setTangent(180)
                .lineToX(-60);

//        TrajectoryActionBuilder spikeThree = drive.actionBuilder(goalPose)
//                .setTangent(315)
//                .splineToLinearHeading(spikeThreePose, spikeHeading);
//
//        TrajectoryActionBuilder spikeThreeIntake = drive.actionBuilder(spikeThreePose)
//                .lineToY(intakeSpikeY, slow);

        double targetRpm = 3000;

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        mechanism.shooterPowerUp(),
                        start.build(),
                        new SleepAction(.3),

                        mechanism.intakeSlow(),
                        mechanism.transferSlow(),
                        new SleepAction(3),
                        mechanism.transferIn(),
                        new SleepAction(1),
                        mechanism.powerDown(),
                        new SleepAction(.2),

                        spikeThree.build(),
                        mechanism.intakeIn(),
                        mechanism.transferSlowest(),
                        spikeThreeIntake.build(),
                        new SleepAction(.2),
                        mechanism.intakeStop(),

                        mechanism.shooterPowerUp(),
                        goal3.build(),
                        new SleepAction(.3),

                        mechanism.intakeSlow(),
                        mechanism.transferSlow(),
                        new SleepAction(3),
                        mechanism.transferIn(),
                        new SleepAction(1),
                        mechanism.powerDown(),
                        new SleepAction(.2),

                        mechanism.powerDown(),
                        end.build()
                )
        );
    }
}