package org.firstinspires.ftc.teamcode.LeagueChampThursday;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Disabled
@TeleOp
public class BasicTeleop extends LinearOpMode {

    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx intake, transfer, topShooter, bottomShooter;
    private IMU imu, turretImu;
    private Servo transferGate, shooterRamp;
    private CRServo turret;
    private Limelight3A limelight;
    private ColorSensor topColor, bottomColor;

    /// Boolean values
    private boolean lastA1 = false, lastB1 = false, lastX1 = false, lastY1 = false;
    private boolean lastA2 = false, lastB2 = false, lastX2 = false, lastY2 = false;
    // private boolean a2 = false, b2 = false, x2 = false, y2 = false;
    private boolean active_shooter = false;

    ///  Preset Values
    private final double INT_shooter_power = 0.4, INT_transfer_power = 0.4, INT_intake_power = .85, INT_dt_max_power = 0.8;
    private final double SH_shooter_power = 1, SH_transfer_power = 1, SH_intake_power = .5, SH_dt_max_power = 0.4;
    private double shooter_power = 0, transfer_power = 0, intake_power = 0, dt_max_power = 0;

    private double transferGateOpen = 0.75, transferGateClose = 0.6;

    ///  MODES ///
    private enum POWER_MODE {INTAKE, SHOOT, ENDGAME};
    private POWER_MODE powerMode = POWER_MODE.INTAKE;

    /// Positions
    private Pose2d robot_pose = new Pose2d(0,0,Math.toRadians(0));
    private double blue_goal_x = -70, blue_goal_y = -65;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        initializeServo();

        MecanumDrive drive = new MecanumDrive(hardwareMap, robot_pose);





        waitForStart();
        while (opModeIsActive()){
            switch (powerMode) {

                ///  MODE DETERMINATION
                case INTAKE:
                    intake_power = INT_intake_power;
                    shooter_power = INT_shooter_power;
                    transfer_power = INT_transfer_power;
                    dt_max_power = INT_dt_max_power;

                    transferGate.setPosition(transferGateClose);

                    break;
                case SHOOT:
                    intake_power = SH_intake_power;
                    shooter_power = SH_shooter_power;
                    transfer_power = SH_transfer_power;
                    dt_max_power = SH_dt_max_power;

                    active_shooter = true;

                    // transferGate.setPosition(transferGateOpen);

                    break;
                case ENDGAME:
                    intake_power = 0.85;
                    shooter_power = 1;
                    transfer_power = 1;
                    dt_max_power = 1;
            }


            ///  INTAKE AND TRANSFER
            if (gamepad1.right_bumper){
                intake.setPower(intake_power);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-intake_power);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.right_trigger > 0.2){
                transfer.setPower(transfer_power);
            } else if (gamepad1.left_trigger > 0.2){
                transfer.setPower(-transfer_power);
            } else {
                transfer.setPower(0);
            }


            ///  CHANGE MODES
            if (gamepad2.a && !lastA2){
                powerMode = POWER_MODE.INTAKE;
            }
            if (gamepad2.b && !lastB2){
                powerMode = POWER_MODE.SHOOT;
            }
            if (gamepad2.x && !lastX2){
                powerMode = POWER_MODE.ENDGAME;
            }






            ///  UPDATE GAME PADS ST
            updateGamepads(gamepad1, gamepad2);


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

            ///  Telemetry

        }



    }


    private void updateGamepads (Gamepad gamepad1, Gamepad gamepad2){
        lastA1 = gamepad1.a;
        lastB1 = gamepad1.b;
        lastX1 = gamepad1.x;
        lastY1 = gamepad1.y;

        lastA1 = gamepad2.a;
        lastB1 = gamepad2.b;
        lastX1 = gamepad2.x;
        lastY1 = gamepad2.y;

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
        turretImu = hardwareMap.get(IMU.class, "turretImu");
    }
}
