package org.firstinspires.ftc.teamcode.A_Regional.Autonomous;


import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.A_Regional.C_Intake;
import org.firstinspires.ftc.teamcode.A_Regional.C_Shooter;
import org.firstinspires.ftc.teamcode.A_Regional.C_ShooterTurret;
import org.firstinspires.ftc.teamcode.A_Regional.C_Transfer;
import org.firstinspires.ftc.teamcode.A_Regional.D_BasicTurret;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class A_Blue_Close extends LinearOpMode {
    private DcMotorEx intakeMotor;
    private C_Intake intake;
    private C_ShooterTurret shooterTurret;

    /** TRANSFER **/
    private DcMotorEx transferMotor;
    private Servo transferGate;
    private C_Transfer transfer;

    /** SHOOTER **/
    private DcMotorEx topShooter, bottomShooter;
    private Servo shooterRamp;
    private Servo leftLED, rightLED;

    /** TURRET + LIMELIGHT **/
    private Limelight3A limelight;
    private CRServo turretServo;

    /** Drive (optional, kept for structure) **/
    private MecanumDrive drive;

    // Toggle state
    private boolean lastB2 = false;
    private boolean turretTracking = false;

    private boolean lastX2 = false;
    private boolean shooterEnabled = false;

    private double  distanceIn = 70;
    /// Drive
    private double dt_max_power = 0.9;
    private double targetRpm = 2400, targetRampPos = 0.38;
    private double initX = -52, initY = -52, initHeading = Math.toRadians(225);
    private double goalX = -17, goalY = -17, goalHeading = Math.toRadians(225);
    private double spikeX = -12 /*12*/ , spikeY = -30, spikeHeading = Math.toRadians(270);
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

        initialize_transfer();
        initialize_shooterTurret();
        initialize_intake();

        AutonomousActions act = new AutonomousActions(intake, transfer, shooterTurret);

        Pose2d initialPose = new Pose2d(initX, initY, initHeading);
        drive = new MecanumDrive(hardwareMap, initialPose);

        double spike_x_gap = 24;

        Pose2d goalPose = new Pose2d(goalX, goalY, goalHeading);

        Pose2d spike2Pose = new Pose2d( spikeX + spike_x_gap , spikeY, spikeHeading);
        Pose2d spike1Pose = new Pose2d( spikeX, spikeY, spikeHeading);
        Pose2d intoSpike1 = new Pose2d(spikeX, intakeSpikeY,spikeHeading);
        Pose2d spike3Pose = new Pose2d( spikeX + (2* spike_x_gap), spikeY, spikeHeading);

        Pose2d afterspike2Pose = new Pose2d( spikeX + spike_x_gap, intakeSpikeY, spikeHeading);
        Pose2d gatePose = new Pose2d(gateX, gateY, gateHeading);
        Pose2d afterspike1Pose = new Pose2d( spikeX, intakeSpikeY, spikeHeading);
        Pose2d afterspike3Pose = new Pose2d( spikeX + (2 * spike_x_gap), intakeSpikeY, spikeHeading);

        TranslationalVelConstraint slow = new TranslationalVelConstraint(20);

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
                .splineToLinearHeading(gatePose, spike2Pose.heading.toDouble());

        TrajectoryActionBuilder toGoal2 = drive.actionBuilder(gatePose)
                .setTangent(toShootTangent)//toShootTangent)
                .splineToLinearHeading(goalPose, goalPose.heading.toDouble());

        TrajectoryActionBuilder toGoal2noGate = drive.actionBuilder(afterspike2Pose)
                .setTangent(toShootTangent)
                .splineToLinearHeading(goalPose, goalPose.heading.toDouble());



        TrajectoryActionBuilder toSpike1 = drive.actionBuilder(goalPose)
                .setTangent(leaveGoalTangent)
                .splineToLinearHeading(spike1Pose, spike1Pose.heading.toDouble());

        TrajectoryActionBuilder intSpike1 = drive.actionBuilder(spike1Pose)
                .setTangent(getSpikeTangent)
                .splineToLinearHeading(intoSpike1, spike1Pose.heading.toDouble());

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
                        act.setPipelineA(0),
                        start.build(),
                        act.track(),
                        new SleepAction(.1),
                        new ParallelAction(
                                act.spinFlywheelHold(),

                                new SequentialAction(
                                        new SleepAction(2),
                                        act.int_tf_outtake(),
                                        new SleepAction(1.5),
                                        // start.build()),
                                        act.int_tf_intake(),
                                        new SleepAction(1),

                                        toSpike1.build(),
                                        new SleepAction(.5),
                                        intSpike1.build()
//                                        new SleepAction(.5),
//                                        toGoal1.build(),
//                                        act.track(),
//
//                                        new SleepAction(1),
//                                        act.int_tf_outtake(),
//                                        new SleepAction(2)


                                )






                        )));
    }

    private void initialize_intake() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        intake = new C_Intake(intakeMotor);
    }

    private void initialize_transfer() {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        transferGate  = hardwareMap.get(Servo.class, "transferGate");
        //color = hardwareMap.get(RevColorSensorV3.class, "color");

        // Keep this matching your current constructor usage:
        transfer = new C_Transfer(transferMotor, transferGate, null);
        transfer.deactivate();
    }

    private void initialize_shooterTurret() {
        // Shooter hardware
        topShooter = hardwareMap.get(DcMotorEx.class, "topShooter");
        bottomShooter = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        shooterRamp = hardwareMap.get(Servo.class, "shooterRamp");
        leftLED = hardwareMap.get(Servo.class, "leftLED");
        rightLED = hardwareMap.get(Servo.class, "rightLED");

        // Turret hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(CRServo.class, "turret");

        // Combined class (does all old init/start internally)
        shooterTurret = new C_ShooterTurret(
                topShooter, bottomShooter, shooterRamp, leftLED, rightLED,
                limelight, turretServo
        );

        shooterTurret.setShooterEnabled(false);
        shooterTurret.setTrackingEnabled(false);
    }
}