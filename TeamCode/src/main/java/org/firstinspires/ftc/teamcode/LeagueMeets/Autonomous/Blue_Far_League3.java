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

public class Blue_Far_League3 extends LinearOpMode {
    private double initX = 60, initY = -12, initHeading = Math.toRadians(200);

    private TrajectoryActionBuilder start, spike2, spike1, spike3, gate, goal1, goal2, goal3, leave;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(initX, initY, initHeading);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Mechanism_League3 mechanism = new Mechanism_League3(hardwareMap);

        TranslationalVelConstraint slow = new TranslationalVelConstraint(22);


        TrajectoryActionBuilder leave = drive.actionBuilder(initialPose)
                .setTangent(180)
                .splineToLinearHeading(new Pose2d(60, -40, Math.toRadians(270)), Math.toRadians(270));

        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        mechanism.shooterSetPower(1),
                        mechanism.gateOpen(),
                        new SleepAction(2),
                        mechanism.intakeSlow(),
                        mechanism.transferIn(),
                        new SleepAction(3),
                        mechanism.gateClose(),
                        new SleepAction(.5),
                        mechanism.allStop(),
                        new SleepAction(.1),

                        leave.build()
                )
        );
    }
}