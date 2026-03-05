package org.firstinspires.ftc.teamcode.LeagueChampThursday;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled

public class A_FarZoneRed extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx intake, transfer, topShooter, bottomShooter;
    private IMU imu, turretImu;
    private Servo transferGate, shooterRamp;
    private CRServo turret;
    private Limelight3A limelight;
    private double initX = 0, initY = 0, initHeading = Math.toRadians(90);
    private double zone = 0, goalY = -17, goalHeading = Math.toRadians(270);
    private double zoneX = 0 /*12*/ , zoneY = -7, spikeHeading = Math.toRadians(90);
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
        C_Red_Mechanism mechanism = new C_Red_Mechanism(hardwareMap);

        Pose2d initialPose = new Pose2d(initX, initY, initHeading);
        Pose2d initialPose2 = new Pose2d(initX, initY+38, initHeading);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        double spike_x_gap = 24;

        TranslationalVelConstraint slow = new TranslationalVelConstraint(7);

        ///  Trajectories ///
        TrajectoryActionBuilder start = drive.actionBuilder(initialPose)

                .lineToY(38);
        TrajectoryActionBuilder back = drive.actionBuilder(initialPose2)

                .lineToY(0, new TranslationalVelConstraint(25));
        //TrajectoryActionBuilder
        TrajectoryActionBuilder start2 = drive.actionBuilder(initialPose2)
                .lineToY(40);
        //chickennuggetsandranch
        //T//rajectoryActionBuilder back = drive.actionBuilder(new Pose2d(0,-7, Math.toRadians(180)))
        //     .setTangent(90)
        //     .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(270)), Math.toRadians(180));






        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                mechanism.spinTo(4000, 0.45),
                                mechanism.searchLL()
                        ), new SleepAction(.1),

                        new ParallelAction(
                                mechanism.keepRpm(4000),
                                new SequentialAction(
                                        //mechanism.searchLL(),
                                        mechanism.transfer(),
                                        new SleepAction(2),
                                        mechanism.intake(),

                                        new SleepAction(.1),
                                        start.build(),
                                        mechanism.intake(),
                                        new SleepAction(1),
                                        back.build(),
                                        new SleepAction(.4),
                                        //mechanism.searchLL(),
                                        mechanism.transfer(),
                                        //new SleepAction(1.6),
                                        new SleepAction(1.6),
                                        StorePose.savePos(drive.localizer.getPose())


                                        // new ParallelAction(
                                        //         start.build(),
                                        //             mechanism.intake()
                                        //   ), new SleepAction(.1),
                                        //   back.build(),

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


        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretImu = hardwareMap.get(IMU.class, "turretImu");
        turretImu.resetYaw();
    }
}