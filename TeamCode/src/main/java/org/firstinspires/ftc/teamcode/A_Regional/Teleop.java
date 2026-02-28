package org.firstinspires.ftc.teamcode.A_Regional;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.LeagueChampThursday.S_Adaptive_Equations;
import org.firstinspires.ftc.teamcode.LeagueChampThursday.S_CloseShooterPID;
import org.firstinspires.ftc.teamcode.LeagueChampThursday.StorePose;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class Teleop extends LinearOpMode {

    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx intake, transfer, topShooter, bottomShooter;
    private IMU imu, turretImu;
    private Servo transferGate, shooterRamp;
    private CRServo turret;
    private Limelight3A limelight;
    private ColorSensor topColor, bottomColor;
    private double gateOpen = 0.75, gateClose = 0.61;

    /// Boolean values
    private boolean active_shooter = false;

    ///  Preset Values
    private double shooter_power = 0, transfer_power = 0, intake_power = 0, dt_max_power = 0;

    private double transferGateOpen = 0.75, transferGateClose = 0.65;

    /// Positions
    private Pose2d robot_pose = new Pose2d(-60,-60,Math.toRadians(215));
    private double blue_goal_x = -70, blue_goal_y = -65;

    // X and Y are prob flipped?

    public static double goalX = -70;
    public static double goalY = -70;

    ///  Shooting
    private double targetRPM = 3000;
    private double distanceToGoal = 0;
    private final S_CloseShooterPID shooterPID = new S_CloseShooterPID();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private boolean d2control = false, d2turretcontrol = false;


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        initializeServo();

        MecanumDrive drive = new MecanumDrive(hardwareMap, robot_pose);

        double transferGatePos = .5;
        double shooterRampPos = .5;


        waitForStart();
        while (opModeIsActive()){
            //StorePose sp = new StorePose();
            //telemetry.addData("GetPose", sp.getPose());


            ///  INTAKE AND TRANSFER
            if (gamepad1.right_trigger > 0.2 /*|| gamepad2.right_trigger > 0.2*/) {
                intake.setPower(intake_power *.5);
                transfer.setPower(transfer_power);
               // transferGate.setPosition(gateOpen);
            } else if (gamepad1.right_bumper /*|| gamepad2.right_bumper*/){
                intake.setPower(intake_power);
                transfer.setPower(transfer_power * .7);
                //transferGate.setPosition(gateClose);
            } else if (gamepad1.left_bumper /*|| gamepad2.left_bumper*/) {
                intake.setPower(-intake_power);
                transfer.setPower(-transfer_power);
                //transferGate.setPosition(gateClose);
            } else {
                intake.setPower(0);
                transfer.setPower(0);
                //transferGate.setPosition(gateClose);
            }


//            if (gamepad2.dpadUpWasPressed()){
//                d2control = ! d2control;
//            }
//
//            if (!d2control) {
//                shooterRamp.setPosition(S_Adaptive_Equations.getRampPos(distanceToGoal));
//            } else {
//                if (gamepad2.yWasPressed()) {
//                    shooterRamp.setPosition(Math.max(Math.min(shooterRamp.getPosition() + 0.1, .65), .3));
//                } else if (gamepad2.aWasPressed()) {
//                    shooterRamp.setPosition(Math.max(Math.min(shooterRamp.getPosition() - 0.1, .65), .3));
//                }
//            }




            ///  Shoot
//            if (gamepad1.xWasPressed()) {
//                active_shooter = !active_shooter;
//                shooterPID.reset(); // clean start/stop
//            }

            targetRPM = S_Adaptive_Equations.getFlywheelRPM(distanceToGoal);
            targetRPM = 3100;


            // Clamp target
            if (targetRPM < 0) targetRPM = 0;
            if (targetRPM > S_CloseShooterPID.CFG_maxTargetFlywheelRPM) {
                targetRPM = S_CloseShooterPID.CFG_maxTargetFlywheelRPM;
            }

            shooterPID.setTargetFlywheelRPM(targetRPM);

            // Read encoder velocity (ticks/sec) from topShooter
            double ticksPerSec = topShooter.getVelocity();
            double measuredFlywheelRPM = S_CloseShooterPID.motorTicksPerSecToFlywheelRPM(ticksPerSec);

            double powerCmd;
            if (active_shooter && targetRPM > 0) {
                powerCmd = shooterPID.update(ticksPerSec);
                // transferGate.setPosition(.75);

            } else {
                powerCmd = 0.0;
                // transferGate.setPosition(.6);
            }

            topShooter.setPower(powerCmd);
            bottomShooter.setPower(powerCmd);



            ///  DRIVE
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            Range.clip(-gamepad1.left_stick_y, -dt_max_power, dt_max_power),
                            Range.clip(-gamepad1.left_stick_x, -dt_max_power, dt_max_power)
                    ),
                    Range.clip(-gamepad1.right_stick_x, -dt_max_power, dt_max_power)
            ));


//            if (gamepad2.right_bumper){
//                turret.setPower(-.2);
//            }else if (gamepad2.left_bumper){
//                turret.setPower(.3);
//            } else {
//                turret.setPower(0);
//            }



            if (gamepad1.aWasPressed()){
                transferGatePos -= .03;
            }
            if (gamepad1.bWasPressed()){
                transferGatePos += .03;

            }

            if (gamepad1.xWasPressed()){
                shooterRampPos -= 0.03;
            }
            if (gamepad1.yWasPressed()){
                shooterRampPos += 0.03;
            }

            transferGate.setPosition(transferGatePos);
            shooterRamp.setPosition(shooterRampPos);

            telemetry.addData("shooterRamp", shooterRamp.getPosition());
            telemetry.addData("transferGate", transferGate.getPosition());

            telemetry.update();
        }
    }




    private void initializeServo() {
        turret = hardwareMap.get(CRServo.class, "turret");

        transferGate = hardwareMap.get(Servo.class, "transferGate");
        shooterRamp = hardwareMap.get(Servo.class, "shooterRamp");
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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //turretImu = hardwareMap.get(IMU.class, "turretImu");
        //turretImu.resetYaw();


        shooterPID.reset();
    }
}


