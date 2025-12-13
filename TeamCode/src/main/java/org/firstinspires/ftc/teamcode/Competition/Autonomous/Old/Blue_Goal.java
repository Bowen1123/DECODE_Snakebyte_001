package org.firstinspires.ftc.teamcode.Competition.Autonomous.Old;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition.Mechanism;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class Blue_Goal extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, transfer, shooter, intake;
    private Servo ramp;
    private IMU imu;
    private double SPEED_MULTIPLIER = 1;
    private double RAMP_EXTEND_LIMIT, RAMP_RETRACT_LIMIT;

    private boolean pastA = false, pastB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-52,-52,Math.toRadians(225));

        Pose2d shootPose = new Pose2d(-24,-24,Math.toRadians(225));
        Pose2d prespike1 = new Pose2d(-12,-28, Math.toRadians(270));
        Pose2d postspike1 = new Pose2d(-12,-52, Math.toRadians(270));
        Pose2d prespike2 = new Pose2d(12,-28, Math.toRadians(270));
        Pose2d postspike2 = new Pose2d(12,-52, Math.toRadians(270));
        Pose2d prespike3 = new Pose2d(36,-28, Math.toRadians(270));
        Pose2d postspike3 = new Pose2d(36,-28, Math.toRadians(270));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Mechanism mechanism = new Mechanism(hardwareMap);

        TrajectoryActionBuilder backUp = drive.actionBuilder(initialPose)
                .lineToX(-24);

        TrajectoryActionBuilder toSpike1 = drive.actionBuilder(shootPose)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-12, -28, Math.toRadians(270)), Math.toRadians(0));

        TrajectoryActionBuilder forwardSpike1 = drive.actionBuilder(prespike1)
                .setTangent(Math.toRadians(90))
                .lineToY(-52, new TranslationalVelConstraint(14));

        TrajectoryActionBuilder fromSpike1 = drive.actionBuilder(postspike1)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-24,-24, Math.toRadians(225)), Math.toRadians(180), new TranslationalVelConstraint(44));

        TrajectoryActionBuilder toSpike2 = drive.actionBuilder(shootPose)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(12, -28, Math.toRadians(270)), Math.toRadians(0));

        TrajectoryActionBuilder forwardSpike2 = drive.actionBuilder(prespike2)
                .setTangent(Math.toRadians(90))
                .lineToY(-52, new TranslationalVelConstraint(14));

        TrajectoryActionBuilder fromSpike2 = drive.actionBuilder(postspike2)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-24,-24, Math.toRadians(225)), Math.toRadians(180), new TranslationalVelConstraint(44));

        TrajectoryActionBuilder toSpike3 = drive.actionBuilder(shootPose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(36, -28, Math.toRadians(270)), Math.toRadians(0));


        TrajectoryActionBuilder forwardSpike3 = drive.actionBuilder(prespike3)
                .setTangent(Math.toRadians(90))
                .lineToY(-52, new TranslationalVelConstraint(14));

        TrajectoryActionBuilder fromSpike3 = drive.actionBuilder(postspike3)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-24,-24, Math.toRadians(225)), Math.toRadians(180), new TranslationalVelConstraint(44));

        TrajectoryActionBuilder leave = drive.actionBuilder(shootPose)
                .splineToConstantHeading(new Vector2d(-60, 24), Math.toRadians(225));


        SequentialAction shoot_sequence = new SequentialAction(
                mechanism.intakeSlow(),
                mechanism.transferIn(),
                new SleepAction(.5),

                mechanism.transferOut(),
                new SleepAction(.7),
                mechanism.transferIn(),
                new SleepAction(.7),

                mechanism.transferOut(),
                new SleepAction(.8),
                mechanism.transferInFinal(),
                new SleepAction(.8)
        );

        Action intake = mechanism.intakeIn();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        backUp.build(),
                        mechanism.shooterPowerUp()),
                shoot_sequence,
                mechanism.powerDown(),


                toSpike1.build(),
                intake, mechanism.transferSlowest(),
                forwardSpike1.build(),
                mechanism.powerDownIntake(),

                new ParallelAction(
                        fromSpike1.build(),
                        mechanism.shooterPowerUp()),

                /// SHOOT SEQUENCE ///
                mechanism.intakeSlow(),
                mechanism.transferIn(),
                new SleepAction(.5),

                mechanism.transferOut(),
                new SleepAction(.7),
                mechanism.transferIn(),
                new SleepAction(.7),

                mechanism.transferOut(),
                new SleepAction(.8),
                mechanism.transferInFinal(),
                new SleepAction(.8),
                /// SHOOT SEQUENCE END ///

                mechanism.powerDown(),

                toSpike2.build(),
                intake, mechanism.transferSlowest(),
                forwardSpike2.build(),
                mechanism.powerDownIntake(),

                new ParallelAction(
                        fromSpike2.build(),
                        mechanism.shooterPowerUp()),

                /// SHOOT SEQUENCE ///
                mechanism.intakeSlow(),
                mechanism.transferIn(),
                new SleepAction(.5),

                mechanism.transferOut(),
                new SleepAction(.7),
                mechanism.transferIn(),
                new SleepAction(.7),

                mechanism.transferOut(),
                new SleepAction(.8),
                /// SHOOT SEQUENCE END ///

                mechanism.powerDown(),


                toSpike3.build(),
                intake, mechanism.transferSlowest(),
                forwardSpike3.build(),
                mechanism.powerDownIntake(),

                new ParallelAction(
                        fromSpike3.build(),
                        mechanism.shooterPowerUp()),

                /// SHOOT SEQUENCE ///
                mechanism.intakeSlow(),
                mechanism.transferIn(),
                new SleepAction(.5),

                mechanism.transferOut(),
                new SleepAction(.7),
                mechanism.transferIn(),
                new SleepAction(.7),

                mechanism.transferOut(),
                new SleepAction(.8),
                /// SHOOT SEQUENCE END ///

                mechanism.powerDown(),

                leave.build(),

                new SleepAction(30)
        ));



    }
}
