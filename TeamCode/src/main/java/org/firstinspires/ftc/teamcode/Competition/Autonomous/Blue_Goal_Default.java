package org.firstinspires.ftc.teamcode.Competition.Autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.Mechanism_League2;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class Blue_Goal_Default extends LinearOpMode {
    private double initX = -52, initY = -52, initHeading = Math.toRadians(225);
    private double goalX = -20, goalY = -20, goalHeading = Math.toRadians(215);
    private double spikeX = -12 /*12*/ , spikeY = -52, spikeHeading = Math.toRadians(270);

    private TrajectoryActionBuilder start, spike1, spike2, spike3, gate, goal1, goal2, goal3, leave;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(initX, initY, initHeading);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        double spike_x_gap = 24;

        VelConstraint slow = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -34) {
                return 20.0;
            } else {
                return 50.0;
            }
        };

        Pose2d goalPose = new Pose2d(goalX, goalY, goalHeading);

        Pose2d gatePose = new Pose2d(0, -56, Math.toRadians(180));
        Pose2d spikeOnePose = new Pose2d(spikeX, spikeY, spikeHeading);
        Pose2d spikeTwoPose = new Pose2d(spikeX + spike_x_gap, spikeY, spikeHeading);
        Pose2d spikeThreePose = new Pose2d(spikeX + (2 * spike_x_gap), spikeY, spikeHeading);

        Mechanism_League2 mechanism = new Mechanism_League2(hardwareMap);

        start = drive.actionBuilder(initialPose)
                .lineToX(-20);

        spike1 = drive.actionBuilder(goalPose)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(spikeOnePose, Math.toRadians(265), slow);

        gate = drive.actionBuilder((spikeOnePose))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(0, -56, Math.toRadians(180)), Math.toRadians(270));

        goal1 = drive.actionBuilder(gatePose)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(goalPose, Math.toRadians(90));

        spike2 = drive.actionBuilder(goalPose)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(spikeTwoPose, Math.toRadians(265), slow);

        goal2 = drive.actionBuilder(spikeTwoPose)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(goalPose, Math.toRadians(90));

        spike3 = drive.actionBuilder(goalPose)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(spikeThreePose, Math.toRadians(265), slow);

        goal3 = drive.actionBuilder(spikeThreePose)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(goalPose, Math.toRadians(90));

        leave = drive.actionBuilder(goalPose)
                .setTangent(180)
                .lineToX(-60);


        double targetRpm = 3000;

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                    mechanism.shooterPowerUp(),
                    start.build(),
                    new SleepAction(.2),

                    ///  Shoot Sequence ///
                    mechanism.intakeSlow(),
                    mechanism.transferIn(),
                    new SleepAction(4),
                    mechanism.shooterPowerDown(),
                    // Stop //

                    mechanism.intakeIn(),
                    mechanism.transferSlow(),
                    spike1.build(),
                    mechanism.transferStop(),
                    new SleepAction(.2),

                    new ParallelAction(goal1.build(),
                            mechanism.shooterPowerUp()),
                    new SleepAction(.1),

                    ///  Shoot Sequence ///
                    mechanism.intakeIn(),
                    mechanism.transferIn(), // transfer speed??
                    new SleepAction(4),
                    mechanism.shooterPowerDown(),
                    ///  End ///

                    mechanism.intakeIn(),
                    mechanism.transferSlow(),
                    spike2.build(),
                    mechanism.transferStop(),
                    new SleepAction(.2),

                    new ParallelAction(goal2.build(),
                            mechanism.shooterPowerUp()),
                    new SleepAction(.1),

                    ///  Shoot Sequence ///
                    mechanism.intakeIn(),
                    mechanism.transferIn(),
                    new SleepAction(4),
                    mechanism.shooterPowerDown(),
                    ///  End ///


                    mechanism.intakeIn(),
                    mechanism.transferSlow(),
                    spike3.build(),
                    mechanism.transferStop(),
                    new SleepAction(.2),

                    new ParallelAction(goal3.build(),
                            mechanism.shooterPowerUp()),
                    new SleepAction(.1),

                    ///  Shoot Sequence ///
                    mechanism.intakeIn(),
                    mechanism.transferSlow(),
                    new SleepAction(3),
                    mechanism.shooterPowerDown(),
                    ///  End ///

                    leave.build()
                )
        );
    }
}