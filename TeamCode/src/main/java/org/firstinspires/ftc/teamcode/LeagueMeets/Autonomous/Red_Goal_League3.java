package org.firstinspires.ftc.teamcode.LeagueMeets.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LeagueMeets.Mechanism_League3;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
@Disabled

public class Red_Goal_League3 extends LinearOpMode {
    private double initX = -52, initY = 52, initHeading = Math.toRadians(-225);
    private double goalX = -17, goalY = 17, goalHeading = Math.toRadians(-220);
    private double spikeX = -12 /*12*/ , spikeY = 27, spikeHeading = Math.toRadians(-270);
    private double gateX = 0, gateY = 53, gateHeading = Math.toRadians(-180);
    private double intakeSpikeY = 57, parkingX = -60;

    ///  Tangents ///
    double leaveGoalTangent = -Math.toRadians(315);
    double getSpikeTangent = -Math.toRadians(270);
    double toGateTangent = -Math.toRadians(180);
    double toShootTangent = -Math.toRadians(90);
    double parkingTangent = -Math.toRadians(180);

    private TrajectoryActionBuilder start, spike2, spike1, spike3, gate, goal1, goal2, goal3, leave;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(initX, initY, initHeading);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Mechanism_League3 mechanism = new Mechanism_League3(hardwareMap);

        double spike_x_gap = 24;

        Pose2d goalPose = new Pose2d(goalX, goalY, goalHeading);

        Pose2d spike2Pose = new Pose2d( spikeX + spike_x_gap, spikeY, spikeHeading);
        Pose2d spike1Pose = new Pose2d( spikeX, spikeY, spikeHeading);
        Pose2d spike3Pose = new Pose2d( spikeX + (2* spike_x_gap), spikeY, spikeHeading);

        Pose2d afterspike2Pose = new Pose2d( spikeX + spike_x_gap, spikeY, spikeHeading);
        Pose2d gatePose = new Pose2d(gateX, gateY, gateHeading);
        Pose2d afterspike1Pose = new Pose2d( spikeX, spikeY, spikeHeading);
        Pose2d afterspike3Pose = new Pose2d( spikeX + (2 * spike_x_gap), spikeY, spikeHeading);

        TranslationalVelConstraint slow = new TranslationalVelConstraint(14);


        ///  Trajectories ///
        TrajectoryActionBuilder start = drive.actionBuilder(initialPose)
                .lineToX(goalX);

        TrajectoryActionBuilder toSpike2 = drive.actionBuilder(goalPose)
                .setTangent(leaveGoalTangent)
                .splineToLinearHeading(spike2Pose, spike2Pose.heading.toDouble());

        TrajectoryActionBuilder intSpike2 = drive.actionBuilder(spike2Pose)
                .setTangent(getSpikeTangent)
                .lineToY(intakeSpikeY, slow);

        TrajectoryActionBuilder toGate2 = drive.actionBuilder(afterspike2Pose)
                .setTangent(toGateTangent)
                .splineToLinearHeading(gatePose, gatePose.heading.toDouble());

        TrajectoryActionBuilder toGoal2 = drive.actionBuilder(gatePose)
                .setTangent(toShootTangent)
                .splineToLinearHeading(goalPose, goalPose.heading.toDouble());

        TrajectoryActionBuilder toGoal2noGate = drive.actionBuilder(afterspike2Pose)
                .setTangent(toShootTangent)
                .splineToLinearHeading(goalPose, goalPose.heading.toDouble());




        TrajectoryActionBuilder toSpike1 = drive.actionBuilder(goalPose)
                .setTangent(leaveGoalTangent)
                .splineToLinearHeading(spike1Pose, spike1Pose.heading.toDouble());

        TrajectoryActionBuilder intSpike1 = drive.actionBuilder(spike1Pose)
                .setTangent(getSpikeTangent)
                .lineToY(intakeSpikeY, slow);

        TrajectoryActionBuilder toGoal1 = drive.actionBuilder(afterspike1Pose)
                .setTangent(toShootTangent)
                .splineToLinearHeading(goalPose, goalPose.heading.toDouble());


        TrajectoryActionBuilder toSpike3 = drive.actionBuilder(goalPose)
                .setTangent(leaveGoalTangent)
                .splineToLinearHeading(spike3Pose, spike3Pose.heading.toDouble());

        TrajectoryActionBuilder intSpike3 = drive.actionBuilder(spike3Pose)
                .setTangent(getSpikeTangent)
                .lineToY(intakeSpikeY, slow);

        TrajectoryActionBuilder toGoal3 = drive.actionBuilder(afterspike3Pose)
                .setTangent(toShootTangent)
                .splineToLinearHeading(goalPose, goalPose.heading.toDouble());

        TrajectoryActionBuilder leave = drive.actionBuilder(goalPose)
                .setTangent(parkingTangent)
                .lineToX(parkingX);

        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        mechanism.shooterSetPower(.60),
                        mechanism.gateClose(),

                        start.build(),
                        new SleepAction(.1),

                        mechanism.gateOpen(),
                        mechanism.intakeSlow(),
                        mechanism.transferIn(),
                        new SleepAction(4),
                        mechanism.gateClose(),
                        new SleepAction(.5),
                        mechanism.stopShooter(),

                        mechanism.stopHalf(),

                        toSpike1.build(),

                        mechanism.intakeIn(),
                        mechanism.transferIn(),

                        intSpike1.build(),
                        new SleepAction(.1),
                        mechanism.stopHalf(),
                        mechanism.shooterSetPower(.60),

                        //toGate2.build(),
                        //toGoal2.build(),
                        toGoal1.build(),
                        //toGoal2noGate.build(),

                        mechanism.gateOpen(),
                        mechanism.intakeSlow(),
                        mechanism.transferIn(),
                        new SleepAction(4),
                        mechanism.gateClose(),
                        new SleepAction(.5),
                        mechanism.stopShooter(),

                        toSpike2.build(),
                        intSpike2.build(),

                        mechanism.shooterSetPower(.60),

                        toGoal2.build(),

                        mechanism.gateOpen(),
                        mechanism.intakeSlow(),
                        mechanism.transferIn(),
                        new SleepAction(4),
                        mechanism.gateClose(),
                        new SleepAction(.5),
                        mechanism.stopShooter(),

                        toSpike3.build(),
                        intSpike3.build(),

                        mechanism.shooterSetPower(.60),

                        toGoal3.build(),

                        mechanism.gateOpen(),
                        mechanism.intakeSlow(),
                        mechanism.transferIn(),
                        new SleepAction(4),
                        mechanism.gateClose(),
                        new SleepAction(.5),
                        mechanism.stopShooter(),

                        leave.build()
                )
        );
    }
}