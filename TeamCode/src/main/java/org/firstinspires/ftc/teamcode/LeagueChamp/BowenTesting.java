package org.firstinspires.ftc.teamcode.LeagueChamp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LeagueMeets.Controller_PIDF_Shooter_Close;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class BowenTesting extends LinearOpMode {

    private static final double TICKS_PER_REV = 28.0;
    private int lastEncoderPos = 0;
    private long lastTimeNs = 0;
    private double topShooterRPM = 0;


    private DcMotorEx leftFront, leftBack, rightFront, rightBack, transfer, intake, topShooter, bottomShooter;
    private Servo transferRamp, transferGate;
    private IMU imu;
    private double SPEED_MULTIPLIER = 1;
    private double RAMP_EXTEND_LIMIT, RAMP_RETRACT_LIMIT;
    private boolean pastX = false, pastB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        Pose2d initPos = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPos);

        S_CloseShooterPID shooterPID = new S_CloseShooterPID();

        double shooterPower = 0.9;
        double intakePower = 0.8;
        double transferPower = 1;

        boolean active_shooter = false;
        boolean gate_open = false;

        double topShooterPos = 0;
        double lastTopShooterPos = 0;

        double targetRPM = 2000;
        waitForStart();
        while (opModeIsActive()){



            if (targetRPM < 0) targetRPM = 0;
            if (targetRPM > S_CloseShooterPID.CFG_maxTargetFlywheelRPM) {
                targetRPM = S_CloseShooterPID.CFG_maxTargetFlywheelRPM;
            }

            shooterPID.setTargetFlywheelRPM(targetRPM);

            double ticksPerSec = topShooter.getVelocity();
            double measuredFlywheelRPM = S_CloseShooterPID.motorTicksPerSecToFlywheelRPM(ticksPerSec);

            double powerCmd;
            if (targetRPM > 0) {
                powerCmd = shooterPID.update(ticksPerSec);
            } else {
                powerCmd = 0.0;
            }


            if (gamepad1.x && !pastX){
                active_shooter = !active_shooter;
            }

            if (active_shooter){
                topShooter.setPower(powerCmd);
                bottomShooter.setPower(powerCmd);
                transferGate.setPosition(.9);
            } else {
                topShooter.setPower(0);
                bottomShooter.setPower(0);
                transferGate.setPosition(.7);
            }


            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * .7,
                            -gamepad1.left_stick_x * .7
                    ),
                    -gamepad1.right_stick_x * .8
            ));



            pastX = gamepad1.x;
            telemetry.addData("Top Shooter Power", powerCmd);
            telemetry.addData("Motor RPM: ", (topShooter.getVelocity()));
            telemetry.update();

        }
    }

    private void initialize(){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        topShooter = hardwareMap.get(DcMotorEx.class, "topShooter");
        bottomShooter = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        transferGate = hardwareMap.get(Servo.class, "transferGate");
        // transferRamp = hardwareMap.get(Servo.class, "transferRamp");

        topShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        topShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(params);
        imu.resetYaw();
    }
}
