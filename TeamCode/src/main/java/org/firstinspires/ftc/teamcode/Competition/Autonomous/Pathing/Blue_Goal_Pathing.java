package org.firstinspires.ftc.teamcode.Competition.Autonomous.Pathing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition.Mechanism_League2;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class Blue_Goal_Pathing extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, transfer, shooter, intake;
    private Servo ramp;
    private IMU imu;
    private double SPEED_MULTIPLIER = 1;
    private double RAMP_EXTEND_LIMIT, RAMP_RETRACT_LIMIT;

    private boolean pastA = false, pastB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-52,-52,Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Mechanism_League2 mechanism = new Mechanism_League2(hardwareMap);

        TrajectoryActionBuilder defaultPath = drive.actionBuilder(initialPose)
                ///  START & CYCLE
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

                .lineToX(-60);



        waitForStart();



        Actions.runBlocking(defaultPath.build());


    }
}