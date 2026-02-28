package org.firstinspires.ftc.teamcode.LeagueChampThursday;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

import org.firstinspires.ftc.teamcode.A_Regional.S_CloseShooterPID_Slew;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class A_RegressionTest extends LinearOpMode {

    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx intake, transfer, topShooter, bottomShooter;
    private IMU imu, turretImu;
    private Servo transferGate, shooterRamp;
    private CRServo turret;
    private Limelight3A limelight;
    private ColorSensor topColor, bottomColor;
    private double gateOpen = 0.75, gateClose = 0.615;

    /// Boolean values
    private boolean lastA1 = false, lastB1 = false, lastX1 = false, lastY1 = false;
    private boolean lastA2 = false, lastB2 = false, lastX2 = false, lastY2 = false;
    // private boolean a2 = false, b2 = false, x2 = false, y2 = false;
    private boolean active_shooter = false;

    ///  Preset Values
    private final double INT_shooter_power = 0.4, INT_transfer_power = 0.9, INT_intake_power = .7, INT_dt_max_power = 0.7;
    private final double SH_shooter_power = 1, SH_transfer_power = .8, SH_intake_power = .5, SH_dt_max_power = 0.4;
    private double shooter_power = 0, transfer_power = .8, intake_power = .8, dt_max_power = .7;

    private double transferGateOpen = 0.75, transferGateClose = 0.65;

    ///  MODES ///
    private enum POWER_MODE {INTAKE, SHOOT, ENDGAME};
    private POWER_MODE powerMode = POWER_MODE.INTAKE;

    /// Positions
    private Pose2d robot_pose = new Pose2d(-62,62,Math.toRadians(135));
    private double blue_goal_x = -70, blue_goal_y = -65;

    // X and Y are prob flipped?

    public static double goalX = -70;
    public static double goalY = 70;

    // Odom
    public static double ODOM_kP = .98;
    public static double ODOM_kD = 0.085;
    public static double ODOM_kS_min = 0.02;
    public static double ODOM_kS_max = 0.12;
    public static double ODOM_kS_fullAtDeg = 12.0;
    public static double ODOM_deadbandDeg = 1.0;
    public static double ODOM_MIN_MOVE_POWER = 0.04;
    public static double ODOM_MAX_POWER = 0.45;

    // Limelight
    public static double LL_kP = 0.018;
    public static double LL_kD = 0.00085;
    public static double LL_kS_min = 0.02;
    public static double LL_kS_max = 0.12;
    public static double LL_kS_fullAtDeg = 12.0;
    public static double LL_deadbandDeg = .65;
    public static double LL_MIN_MOVE_POWER = 0.08;
    public static double LL_MAX_POWER = 0.45;

    // Turret IMU
    public static double turretOffsetRad = 0.0;

    // Limelight
    public static int PIPELINE = 1;
    public static int pollRateHz = 100;

    // tolerance
    public static double limelightArmDeg = 1.4;
    public static double llLostTimeoutSec = 0.15;
    ///  TRACKING
    private boolean trackingOn = false;
    private enum Tracking { LOCK_TO_DRIVE, ODOMETRY, LIMELIGHT }
    private Tracking mode = Tracking.LOCK_TO_DRIVE;
    private double lastErrorRad = 0.0;
    private long lastTimeNs = 0;

    private boolean limelightOn = false;
    private long lastTargetSeenNs = 0;

    ///  Shooting
    private double targetRPM = 2000;
    private double distanceToGoal = 0;
    private final S_CloseShooterPID_Slew shooterPID = new S_CloseShooterPID_Slew();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        initializeServo();
        Pose2d robotPos = new Pose2d(0,0,Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, robotPos);
        Pose2d goalPos = new Pose2d(-10,-10, 0);


        double dist = Math.sqrt(Math.pow(5, 2) + Math.pow(5, 2));
        double rampPos = .3;

        waitForStart();
        while(opModeIsActive()){
            drive.updatePoseEstimate();
            robotPos = drive.localizer.getPose();
            dist = Math.sqrt(Math.pow(goalPos.position.x - robotPos.position.x, 2) + Math.pow(goalPos.position.y - robotPos.position.y, 2));


            if (gamepad1.right_trigger > 0.2 /*|| gamepad2.right_trigger > 0.2*/) {
                intake.setPower(intake_power);
                transfer.setPower(transfer_power);
                transferGate.setPosition(gateOpen);
            } else if (gamepad1.right_bumper /*|| gamepad2.right_bumper*/){
                intake.setPower(intake_power);
                transfer.setPower(transfer_power);
                transferGate.setPosition(gateClose);
            } else if (gamepad1.left_bumper /*|| gamepad2.left_bumper*/) {
                intake.setPower(-intake_power);
                transfer.setPower(-transfer_power);
                transferGate.setPosition(gateClose);
            } else {
                intake.setPower(0);
                transfer.setPower(0);
                transferGate.setPosition(gateClose);
            }



            if (gamepad1.aWasPressed()) targetRPM += 100.0;
            if (gamepad1.bWasPressed()) targetRPM -= 100.0;
            ///  Shoot
            if (gamepad1.xWasPressed()) {
                active_shooter = !active_shooter;
                shooterPID.reset(); // clean start/stop
            }

            if (gamepad1.dpadDownWasPressed()){
                rampPos -= 0.05;
            }
            if (gamepad1.dpadUpWasPressed()){
                rampPos += 0.05;
            }
            shooterRamp.setPosition(Math.max(Math.min(rampPos, 0.75), 0.3));

            // targetRPM = S_Adaptive_Equations.getFlywheelRPM(distanceToGoal);

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


            ///  Telemetry
            telemetry.addData("Robot Position: ", "("+ pose.position.x +", " + pose.position.y + ")");
            telemetry.addData("Distance: ", dist);

            double motorRPM = topShooter.getVelocity() * 60 / 28;

            telemetry.addData("Targget Flywheel RPM: ", targetRPM);
            telemetry.addData("Motor Power: ", powerCmd);
            telemetry.addData("Motor RPM: ", motorRPM);
            telemetry.addData("Flywheel RPM: ", motorRPM * 1.5);

            telemetry.addData("Ramp Position", rampPos);


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
        turretImu = hardwareMap.get(IMU.class, "turretImu");
        turretImu.resetYaw();



        shooterPID.reset();
    }
}
