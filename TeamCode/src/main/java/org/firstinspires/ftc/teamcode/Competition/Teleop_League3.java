package org.firstinspires.ftc.teamcode.Competition;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class Teleop_League3 extends LinearOpMode {

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

        double intakePower = 0.8;
        double transferPower = 1;

        boolean active_shooter = false;
        Controller_PIDF_Shooter_Close shooter_pid = new Controller_PIDF_Shooter_Close(topShooter);

        waitForStart();
        while (opModeIsActive()){
            /// ----------------- Mechanism Controls -----------------

            if (gamepad2.x){
                active_shooter = true;
                shooter_pid.setTargetRpm(2000);
            } else  if (gamepad2.y){
                active_shooter = false;
            }

            if (active_shooter){
//                double power = shooter_pid.update();
//                topShooter.setPower(power);
//                bottomShooter.setPower(power);
                topShooter.setPower(.8);
                bottomShooter.setPower(.8);

            } else {
                topShooter.setPower(0);
                bottomShooter.setPower(0);
            }

            if (gamepad2.a){
                transferGate.setPosition(.55);
            }
            if (gamepad2.b){
                transferGate.setPosition(.25);
            }

            if (gamepad2.dpad_up){
                transferRamp.setPosition(1);
            }
            if (gamepad2.dpad_down){
                transferRamp.setPosition(.7);
            }

            if (gamepad2.right_bumper){
                intake.setPower(intakePower);
            } else if (gamepad2.right_trigger > .2) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            if (gamepad2.left_bumper){
                transfer.setPower(transferPower);
            } else if (gamepad2.left_trigger > .2) {
                transfer.setPower(-transferPower);
            } else {
                transfer.setPower(0);
            }

            pastA = gamepad2.a;
            pastB = gamepad2.b;

            ///  Drive ///
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
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
        transferRamp = hardwareMap.get(Servo.class, "transferRamp");

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
