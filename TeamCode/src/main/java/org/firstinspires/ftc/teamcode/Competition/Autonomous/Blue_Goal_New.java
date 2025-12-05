package org.firstinspires.ftc.teamcode.Competition.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition.Mechanism;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class Blue_Goal_New extends LinearOpMode {


    private double initX = -52, initY = -52, initHeading = Math.toRadians(225);
    private double goalX = -24, goalY = -24, goalHeading = Math.toRadians(225);
    private double spikeX = -12, spikeY = -52, spikeHeading = Math.toRadians(270);

    private TrajectoryActionBuilder start, spike1, spike2, spike3, goal1, goal2, goal3, leave;


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(initX, initY, initHeading);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        double spike_x_gap = 24;
        double spike_y_gap = 28;

        VelConstraint slow = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -34) {
                return 20.0;
            } else {
                return 50.0;
            }
        };

        Pose2d goalPose = new Pose2d(goalX, goalY, goalHeading);

        Pose2d spikeOnePose = new Pose2d(spikeX, spikeY, spikeHeading);
        Pose2d spikeTwoPose = new Pose2d(spikeX + spike_x_gap, spikeY, spikeHeading);
        Pose2d spikeThreePose = new Pose2d(spikeX + (2 * spike_x_gap), spikeY, spikeHeading);


        Mechanism mechanism = new Mechanism(hardwareMap);

        start = drive.actionBuilder(initialPose)
                .lineToX(-24);

        spike1 = drive.actionBuilder(goalPose)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(spikeOnePose, Math.toRadians(265), slow);

        goal1 = drive.actionBuilder(spikeOnePose)
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




        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                    start.build(),
                    spike1.build(),
                    goal1.build(),
                    spike2.build(),
                    goal2.build(),
                    spike3.build(),
                    goal3.build(),
                    leave.build()
                )
        );
    }
}