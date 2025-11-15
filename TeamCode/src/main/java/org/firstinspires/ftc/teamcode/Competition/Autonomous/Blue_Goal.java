package org.firstinspires.ftc.teamcode.Competition.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition.Mechanism;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class Blue_Goal extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, transfer, shooter, intake;
    private Servo ramp;
    private IMU imu;
    private double SPEED_MULTIPLIER = 1;
    private double RAMP_EXTEND_LIMIT, RAMP_RETRACT_LIMIT;

    private boolean pastA = false, pastB = false;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(45));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Mechanism mechanism = new Mechanism(hardwareMap);

        TrajectoryActionBuilder defaultPath = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(18, -12), Math.toRadians(315));

        TrajectoryActionBuilder backUp = drive.actionBuilder(initialPose)
                .lineToX(-20);

        TrajectoryActionBuilder leave = drive.actionBuilder(new Pose2d(-20, -20, Math.toRadians(45)))
                .turn(Math.toRadians(135))
                .lineToX(-44);
        // .splineTo(new Vector2d(-24, 24), Math.toRadians(90));

        TrajectoryActionBuilder spike1 = drive.actionBuilder(new Pose2d(-44, 20, Math.toRadians(180)))
                .turn(Math.toRadians(90))
                .lineToY(16);


        waitForStart();

        Actions.runBlocking(new SequentialAction(
                mechanism.shooterPowerUp(),
                backUp.build(),
                new SleepAction(1.4),
                mechanism.transferSlow(),
                new SleepAction(2),

                //mechanism.transferStop(),
                mechanism.intakeIn(),
                new SleepAction(.5),
                mechanism.intakeStop(),
                mechanism.transferSlow(),
                new SleepAction(2),

                mechanism.intakeIn(),
                new SleepAction(.5),
                ///mechanism.intakeStop(),
                mechanism.transferOut(),
                new SleepAction(.3),
                mechanism.transferIn(),
                //mechanism.transferSlow(),
                new SleepAction(1.2),

                mechanism.powerDown(),
                leave.build(),
                new SleepAction(.5),

                ///mechanism.intakeIn(),


                new SleepAction(30)
        ));



    }

    private void moveRobot(double x, double y, double yaw) {
        double fl =  y - x - yaw;
        double fr =  y + x + yaw;
        double bl =  y + x - yaw;
        double br =  y - x + yaw;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        leftFront.setPower(fl * SPEED_MULTIPLIER);
        rightFront.setPower(fr * SPEED_MULTIPLIER);
        leftBack.setPower(bl * SPEED_MULTIPLIER);
        rightBack.setPower(br * SPEED_MULTIPLIER);
    }
    private void initialize(){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        //shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu.initialize(params);
        imu.resetYaw();


    }
}
