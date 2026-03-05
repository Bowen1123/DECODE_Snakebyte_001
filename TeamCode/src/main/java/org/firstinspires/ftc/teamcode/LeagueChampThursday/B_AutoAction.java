package org.firstinspires.ftc.teamcode.LeagueChampThursday;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
@Disabled


public class B_AutoAction extends LinearOpMode {

    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx intake, transfer, topShooter, bottomShooter;
    private IMU imu, turretImu;
    private Servo transferGate, shooterRamp;
    private CRServo turret;
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        initializeServo();
        // C_Outtake outtake = new C_Outtake(turret, limelight, topShooter, bottomShooter);
        C_Blue_Mechanism mechanism = new C_Blue_Mechanism(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(20,20, Math.toRadians(90)));



        waitForStart();
//        Actions.runBlocking(outtake.spinToRpm(3000, 100, 3, 5));
        Actions.runBlocking(
                StorePose.savePos(drive.localizer.getPose())

        );
//        Actions.runBlocking(outtake.searchAndAimAprilTag(3, 1, 2));

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
