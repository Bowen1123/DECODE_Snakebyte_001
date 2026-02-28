package org.firstinspires.ftc.teamcode.LeagueChampThursday;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LeagueChampThursday.C_Blue_Mechanism;
import org.firstinspires.ftc.teamcode.LeagueMeets.Mechanism_League3;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class A_Blue_Goal_Auto extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx intake, transfer, topShooter, bottomShooter;
    private IMU imu, turretImu;
    private Servo transferGate, shooterRamp;
    private CRServo turret;
    private Limelight3A limelight;
    private double initX = -52, initY = -52, initHeading = Math.toRadians(225);
    private double goalX = -17, goalY = -17, goalHeading = Math.toRadians(225);
    private double spikeX = -12 /*12*/ , spikeY = -36, spikeHeading = Math.toRadians(270);
    private double gateX = 0, gateY = -53, gateHeading = Math.toRadians(180);
    private double intakeSpikeY = -64, parkingX = -60;

    ///  Tangents ///
    double leaveGoalTangent = Math.toRadians(315);
    double getSpikeTangent = Math.toRadians(270);
    double toGateTangent = Math.toRadians(180);
    double toShootTangent = Math.toRadians(90);
    double parkingTangent = Math.toRadians(180);

    private TrajectoryActionBuilder start, spike2, spike1, spike3, gate, goal1, goal2, goal3, leave;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        initializeServo();
        // C_Outtake outtake = new C_Outtake(turret, limelight, topShooter, bottomShooter);
        C_Blue_Mechanism mechanism = new C_Blue_Mechanism(hardwareMap);

        Pose2d initialPose = new Pose2d(initX, initY, initHeading);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        double spike_x_gap = 24;

        Pose2d goalPose = new Pose2d(goalX, goalY, goalHeading);

        Pose2d spike2Pose = new Pose2d( spikeX + spike_x_gap - 1, spikeY, spikeHeading);
        Pose2d spike1Pose = new Pose2d( spikeX, spikeY, spikeHeading);
        Pose2d spike3Pose = new Pose2d( spikeX + (2* spike_x_gap), spikeY, spikeHeading);

        Pose2d afterspike2Pose = new Pose2d( spikeX + spike_x_gap, spikeY * 1.1, spikeHeading);
        Pose2d gatePose = new Pose2d(gateX, gateY, gateHeading);
        Pose2d afterspike1Pose = new Pose2d( spikeX, spikeY, spikeHeading);
        Pose2d afterspike3Pose = new Pose2d( spikeX + (2 * spike_x_gap), spikeY, spikeHeading);

        TranslationalVelConstraint slow = new TranslationalVelConstraint(12);


        ///  Trajectories ///
        TrajectoryActionBuilder start = drive.actionBuilder(initialPose)
                .lineToX(goalX);

        TrajectoryActionBuilder toSpike2 = drive.actionBuilder(goalPose)
                .setTangent(Math.toRadians(leaveGoalTangent))
                .splineToLinearHeading(spike2Pose, spike2Pose.heading.toDouble());

        TrajectoryActionBuilder intSpike2 = drive.actionBuilder(spike2Pose)
                .setTangent(getSpikeTangent)
                .lineToY(intakeSpikeY, slow);

        TrajectoryActionBuilder toGate2 = drive.actionBuilder(afterspike2Pose)
                .setTangent(toGateTangent)
                .splineToLinearHeading(gatePose, spike2Pose.heading.toDouble()); //change #1 gatePose.heading.toDouble() --> spike2Pose.heading.toDouble()

        TrajectoryActionBuilder toGoal2 = drive.actionBuilder(gatePose)
                .setTangent(Math.toRadians(270))//toShootTangent)
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
                    new ParallelAction(
                            mechanism.spinTo(3100, 0.36),
                            mechanism.searchLL(),
                            start.build()
                    ), new SleepAction(.1),

                    new ParallelAction(
                            mechanism.keepRpm(3000),
                            new SequentialAction(
                                    mechanism.transfer(),
                                    new SleepAction(1.6),
                                    mechanism.intake(),
                                    toSpike2.build(),
                                    intSpike2.build(),
                                    new SleepAction(.1),

                                    //toGate2.build(),

                                    new ParallelAction(
                                            toGoal2noGate.build(),
                                            mechanism.searchLL()
                                    ), new SleepAction(.1),

                                    mechanism.transfer(),
                                    new SleepAction(1.6),
                                    mechanism.intake(),
                                    toSpike1.build(),
                                    intSpike1.build(),
                                    new SleepAction(.1),

                                    new ParallelAction(
                                            toGoal1.build(),
                                            mechanism.searchLL()
                                    ), new SleepAction(.1),

                                    mechanism.transfer(),
                                    new SleepAction(1.6),
                                    mechanism.stop(),
                                    new SleepAction(.1),
                                    leave.build(),
                                    StorePose.savePos(drive.localizer.getPose())
                    )
                )
        ));


    }


    private void initializeServo() {
        turret = hardwareMap.get(CRServo.class, "turret");

        transferGate = hardwareMap.get(Servo.class, "transferGate");
        shooterRamp = hardwareMap.get(Servo.class, "shooterRamp");
    }

    private void initialize(){
        topShooter = hardwareMap.get(DcMotorEx.class, "topShooter");
        bottomShooter = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        topShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);


        topShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //turretImu = hardwareMap.get(IMU.class, "turretImu");
        //turretImu.resetYaw();
    }
}