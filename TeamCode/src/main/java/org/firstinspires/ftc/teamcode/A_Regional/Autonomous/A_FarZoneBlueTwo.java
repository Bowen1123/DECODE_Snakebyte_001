package org.firstinspires.ftc.teamcode.A_Regional.Autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
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
public class A_FarZoneBlueTwo extends LinearOpMode {
    private DcMotorEx intakeMotor;
    private C_Intake intake;
    private C_ShooterTurret shooterTurret;

    /** TRANSFER **/
    private DcMotorEx transferMotor;
    private Servo transferGate;
    private RevColorSensorV3 color;
    private C_Transfer transfer;

    /** SHOOTER **/
    private DcMotorEx topShooter, bottomShooter;
    private Servo shooterRamp;
    private C_Shooter shooter;
    private Servo leftLED, rightLED;

    /** TURRET + LIMELIGHT **/
    private Limelight3A limelight;
    private CRServo turretServo;
    private D_BasicTurret turret;

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

        initialize_transfer();
        initialize_shooterTurret();
        initialize_intake();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        shooter.setTestTargetRampPos(targetRampPos);
        shooter.setTestTargetRPM(targetRpm);
        Pose2d initialPose = new Pose2d(initX, initY, initHeading);
        Pose2d initialPose2 = new Pose2d(initX, initY+38, initHeading);
        // MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        double spike_x_gap = 24;



        TranslationalVelConstraint slow = new TranslationalVelConstraint(7);


        ///  Trajectories ///
        TrajectoryActionBuilder start = drive.actionBuilder(initialPose)

                .lineToY(38);

        TrajectoryActionBuilder back = drive.actionBuilder(initialPose2)

                .lineToY(-2, new TranslationalVelConstraint(25));
        //TrajectoryActionBuilder
        TrajectoryActionBuilder start2 = drive.actionBuilder(initialPose2)
                .lineToY(40);




        //T//rajectoryActionBuilder back = drive.actionBuilder(new Pose2d(0,-7, Math.toRadians(180)))
        //     .setTangent(90)
        //     .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(270)), Math.toRadians(180));






        waitForStart();
        Actions.runBlocking(
                new ParallelAction(

                ));
    }
    private void initializeServo() {
        turret = (D_BasicTurret) hardwareMap.get(CRServo.class, "turret");

        transferGate = hardwareMap.get(Servo.class, "transferGate");
        shooterRamp = hardwareMap.get(Servo.class, "shooterRamp");
    }

    private void initialize_intake() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        intake = new C_Intake(intakeMotor);
    }

    private void initialize_transfer() {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        transferGate  = hardwareMap.get(Servo.class, "transferGate");
        color = hardwareMap.get(RevColorSensorV3.class, "color");

        // Keep this matching your current constructor usage:
        transfer = new C_Transfer(transferMotor, transferGate, color);
        transfer.deactivate();
    }

    private void initialize_shooterTurret() {
        // Shooter hardware
        DcMotorEx topShooter = hardwareMap.get(DcMotorEx.class, "topShooter");
        DcMotorEx bottomShooter = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        Servo shooterRamp = hardwareMap.get(Servo.class, "shooterRamp");
        Servo leftLED = hardwareMap.get(Servo.class, "leftLED");
        Servo rightLED = hardwareMap.get(Servo.class, "rightLED");

        // Turret hardware
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        CRServo turretServo = hardwareMap.get(CRServo.class, "turret");

        // Combined class (does all old init/start internally)
        shooterTurret = new C_ShooterTurret(
                topShooter, bottomShooter, shooterRamp, leftLED, rightLED,
                limelight, turretServo
        );

        shooterTurret.setShooterEnabled(false);
        shooterTurret.setTrackingEnabled(false);
    }
}