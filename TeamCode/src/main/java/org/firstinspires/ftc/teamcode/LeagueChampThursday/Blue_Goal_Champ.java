package org.firstinspires.ftc.teamcode.LeagueMeets.Autonomous;

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
public class Blue_Goal_Champ extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx intake, transfer, topShooter, bottomShooter;
    private IMU imu, turretImu;
    private Servo transferGate, shooterRamp;
    private CRServo turret;
    private Limelight3A limelight;
    private double initX = -52, initY = -52, initHeading = Math.toRadians(225);
    private double goalX = -17, goalY = -17, goalHeading = Math.toRadians(227);
    private double spikeX = -12 /*12*/ , spikeY = -52, spikeHeading = Math.toRadians(270);
    private double gateX = 0, gateY = -53, gateHeading = Math.toRadians(180);
    private double intakeSpikeY = -58, parkingX = -60;

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

        double intakeTurn = Math.toRadians(15);
        Pose2d spike2Pose = new Pose2d( spikeX + spike_x_gap, spikeY, spikeHeading + intakeTurn);
        Pose2d spike1Pose = new Pose2d( spikeX, spikeY, spikeHeading);




        TranslationalVelConstraint slow = new TranslationalVelConstraint(12);


        ///  Trajectories ///
        TrajectoryActionBuilder start = drive.actionBuilder(initialPose)
                .lineToX(goalX);

        TrajectoryActionBuilder toSpike2 = drive.actionBuilder(goalPose)
                .setTangent(0)
                .splineToLinearHeading(spike2Pose, 270);

        TrajectoryActionBuilder toGoal2 = drive.actionBuilder(spike2Pose)
                .setTangent(toGateTangent)
                .splineToLinearHeading(goalPose, goalPose.heading.toDouble());


        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                    new ParallelAction(
                            start.build(),
                            mechanism.spinTo(3000, 0.4),
                            mechanism.searchLL()
                    ),
                    new SleepAction(.2),
                    mechanism.transfer(),
                    new SleepAction(2),

                        new ParallelAction(
                                mechanism.intake(),
                                toSpike2.build()
                        ),
                        new SleepAction(.1),
                        new ParallelAction(
                                toGoal2.build(),
                                mechanism.spinTo(3000, 0.4),
                                    new SequentialAction(
                                          new SleepAction(1),
                                          mechanism.searchLL()
                                    )
                                )


                )
        );
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
        turretImu = hardwareMap.get(IMU.class, "turretImu");
        turretImu.resetYaw();
    }
}