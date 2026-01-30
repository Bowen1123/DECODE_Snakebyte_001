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
@Disabled
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
    private boolean pastA = false, pastB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        Pose2d initPos = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPos);

        double shooterPower = 0.9;
        double intakePower = 0.8;
        double transferPower = 1;

        boolean active_shooter = false;
        boolean gate_open = false;

        double topShooterPos = 0;
        double lastTopShooterPos = 0;

        waitForStart();
        while (opModeIsActive()){
            topShooterPos = topShooter.getCurrentPosition();

            long now = System.nanoTime();
            int currentPos = topShooter.getCurrentPosition();

            if (lastTimeNs != 0) {
                double deltaTimeSec = (now - lastTimeNs) / 1e9;
                int deltaTicks = currentPos - lastEncoderPos;

                double revs = deltaTicks / TICKS_PER_REV;
                topShooterRPM = (revs / deltaTimeSec) * 60.0;
            }

            lastEncoderPos = currentPos;
            lastTimeNs = now;

            if (gamepad1.x){
                active_shooter = true;
            } else if (gamepad1.y) {
                active_shooter = false;
            }

            if (active_shooter) {
                topShooter.setPower(1);
                bottomShooter.setPower(1);
            } else {
                topShooter.setPower(0);
                bottomShooter.setPower(0);
            }

            if (gamepad1.right_bumper){
                intake.setPower(.75);
            } else if (gamepad1.left_bumper){
                intake.setPower(-.75);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.right_trigger > 0.2){
                transfer.setPower(1);
            } else if (gamepad1.left_trigger > 0.2){
                transfer.setPower(-.5);
            } else {
                transfer.setPower(0);
            }




            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * .7,
                            -gamepad1.left_stick_x * .7
                    ),
                    -gamepad1.right_stick_x * .8
            ));

            telemetry.addData("Top Shooter RPM", topShooterRPM);
            telemetry.addData("Flywheel RPM: ", topShooterRPM * 1.5);
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
