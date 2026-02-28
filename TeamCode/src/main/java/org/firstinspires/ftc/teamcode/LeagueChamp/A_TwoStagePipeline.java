package org.firstinspires.ftc.teamcode.LeagueChamp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@TeleOp
@Disabled

public class A_TwoStagePipeline extends LinearOpMode {

    public static double goalX = 51.0;
    public static double goalY = 41.0;

    // Odom
    public static double ODOM_kP = 1.2;
    public static double ODOM_kD = 0.05;
    public static double ODOM_kS_min = 0.03;
    public static double ODOM_kS_max = 0.12;
    public static double ODOM_kS_fullAtDeg = 12.0;
    public static double ODOM_deadbandDeg = 1.0;
    public static double ODOM_MIN_MOVE_POWER = 0.1;
    public static double ODOM_MAX_POWER = 0.60;

    // Limelight
    public static double LL_kP = 0.02;
    public static double LL_kD = 0.0005;
    public static double LL_kS_min = 0.02;
    public static double LL_kS_max = 0.12;
    public static double LL_kS_fullAtDeg = 12.0;
    public static double LL_deadbandDeg = .65;
    public static double LL_MIN_MOVE_POWER = 0.1;
    public static double LL_MAX_POWER = 0.52;

    // Turret IMU
    public static double turretOffsetRad = 0.0;

    // Limelight
    public static int PIPELINE = 0;
    public static int pollRateHz = 100;

    // tolerance
    public static double limelightArmDeg = 1.4;
    public static double llLostTimeoutSec = 0.15;

    private CRServo turret;
    private IMU turretImu;
    private Limelight3A limelight;

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean trackingOn = false;

    private enum Tracking { LOCK_TO_DRIVE, ODOMETRY, LIMELIGHT }
    private Tracking mode = Tracking.LOCK_TO_DRIVE;

    private double lastErrorRad = 0.0;
    private long lastTimeNs = 0;

    private boolean limelightOn = false;
    private long lastTargetSeenNs = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        turret = hardwareMap.get(CRServo.class, "turret");
        turretImu = hardwareMap.get(IMU.class, "turretImu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turretImu.resetYaw();
        telemetry.setMsTransmissionInterval(11);

        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            Range.clip(-gamepad1.left_stick_y, -0.6, 0.6),
                            Range.clip(-gamepad1.left_stick_x, -0.6, 0.6)
                    ),
                    Range.clip(-gamepad1.right_stick_x, -0.6, 0.6)
            ));

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            if (a && !lastA) {
                trackingOn = !trackingOn;

                if (trackingOn) {
                    mode = Tracking.ODOMETRY;
                } else {
                    mode = Tracking.LOCK_TO_DRIVE;
                }

                resetDerivative();

                if (!trackingOn) {
                    stopLimelight();
                }
            }
            lastA = a;

            if (b && !lastB) {
                trackingOn = false;
                mode = Tracking.LOCK_TO_DRIVE;
                resetDerivative();
                stopLimelight();
            }
            lastB = b;

            double robotX = pose.position.x;
            double robotY = pose.position.y;
            double robotHeadingRad = pose.heading.toDouble();

            double turretYawRad = getTurretYawRad() + turretOffsetRad;

            double targetAngleRad = Math.atan2(goalY - robotY, goalX - robotX);
            double distanceToGoal = Math.hypot(goalX - robotX, goalY - robotY);

            if (!trackingOn) {
                mode = Tracking.LOCK_TO_DRIVE;
            } else {
                if (mode == Tracking.ODOMETRY) {
                    double errorRad = wrapAngle(targetAngleRad - turretYawRad);
                    double errorDeg = Math.toDegrees(Math.abs(errorRad));

                    if (errorDeg <= limelightArmDeg) {
                        startLimelight();

                        LLResult result = limelight.getLatestResult();
                        if (result != null && result.isValid()) {
                            mode = Tracking.LIMELIGHT;
                            resetDerivative();
                            lastTargetSeenNs = System.nanoTime();
                        }
                    }
                } else if (mode == Tracking.LIMELIGHT) {
                    startLimelight();

                    LLResult result = limelight.getLatestResult();
                    if (result != null && result.isValid()) {
                        lastTargetSeenNs = System.nanoTime();
                    } else {
                        double secondsSinceSeen = (System.nanoTime() - lastTargetSeenNs) / 1e9;
                        if (secondsSinceSeen > llLostTimeoutSec) {
                            mode = Tracking.ODOMETRY;
                            resetDerivative();
                        }
                    }
                }
            }

            double turretPower = 0.0;

            if (mode == Tracking.LOCK_TO_DRIVE) {

                double errorRad = wrapAngle(robotHeadingRad - turretYawRad);
                double errorRate = derivative(errorRad);
                turretPower = odomCmd(errorRad, errorRate);

            } else if (mode == Tracking.ODOMETRY) {

                double errorRad = wrapAngle(targetAngleRad - turretYawRad);
                double errorRate = derivative(errorRad);
                turretPower = odomCmd(errorRad, errorRate);

            } else if (mode == Tracking.LIMELIGHT) {

                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {

                    double errorRad = Math.toRadians(-result.getTx());
                    double errorRate = derivative(errorRad);
                    turretPower = limelightCmd(errorRad, errorRate);

                } else {

                    double errorRad = wrapAngle(targetAngleRad - turretYawRad);
                    double errorRate = derivative(errorRad);
                    turretPower = odomCmd(errorRad, errorRate);

                }

            }

            turret.setPower(turretPower);

            telemetry.addData("Tracking", trackingOn);
            telemetry.addData("Mode", mode);

            // localization telemtry
            telemetry.addData("Robot (x,y)", "(%.2f, %.2f)", robotX, robotY);
            telemetry.addData("Goal (x,y)", "(%.2f, %.2f)", goalX, goalY);
            telemetry.addData("OdomDist", "%.2f", distanceToGoal);

            telemetry.addData("RobotHeadingDeg", "%.2f", Math.toDegrees(robotHeadingRad));
            telemetry.addData("TurretYawDeg", "%.2f", Math.toDegrees(turretYawRad));
            telemetry.addData("TurretPower", "%.3f", turretPower);

            if (limelightOn) {
                LLStatus status = limelight.getStatus();
                telemetry.addData("LL",
                        "Temp: %.1fC | CPU: %.1f%% | FPS: %d | Pipe: %d (%s)",
                        status.getTemp(), status.getCpu(), (int) status.getFps(),
                        status.getPipelineIndex(), status.getPipelineType());

                LLResult result = limelight.getLatestResult();
                boolean hasTarget = false;
                if (result != null && result.isValid()) {
                    hasTarget = true;
                }
                telemetry.addData("LL Has Target", hasTarget);

                if (hasTarget) {
                    telemetry.addData("LL tx (deg)", "%.2f", result.getTx());
                }
            } else {
                telemetry.addData("LL", "Stopped");
            }

            telemetry.update();
        }

//        turret.setPower(0.0);
//        stopLimelight();
    }

    private double odomCmd(double errorRad, double errorRateRadPerSec) {
        double absErr = Math.abs(errorRad);
        double deadbandRad = Math.toRadians(ODOM_deadbandDeg);
        if (absErr <= deadbandRad) return 0.0;

        double fullAtRad = Math.toRadians(ODOM_kS_fullAtDeg);
        double t = Range.clip(absErr / fullAtRad, 0.0, 1.0);
        double smooth = (3.0 * t * t) - (2.0 * t * t * t);
        double kS = ODOM_kS_min + (ODOM_kS_max - ODOM_kS_min) * smooth;

        double power = (ODOM_kP * errorRad) + (ODOM_kD * errorRateRadPerSec) + (kS * Math.signum(errorRad));

        if (Math.abs(power) < ODOM_MIN_MOVE_POWER) {
            power = ODOM_MIN_MOVE_POWER * Math.signum(errorRad);
        }

        return Range.clip(power, -ODOM_MAX_POWER, ODOM_MAX_POWER);
    }

    private double limelightCmd(double errorRad, double errorRateRadPerSec) {
        double absErr = Math.abs(errorRad);
        double deadbandRad = Math.toRadians(LL_deadbandDeg);
        if (absErr <= deadbandRad) return 0.0;

        double fullAtRad = Math.toRadians(LL_kS_fullAtDeg);
        double t = Range.clip(absErr / fullAtRad, 0.0, 1.0);
        double smooth = (3.0 * t * t) - (2.0 * t * t * t);
        double kS = LL_kS_min + (LL_kS_max - LL_kS_min) * smooth;

        double power = (LL_kP * errorRad) + (LL_kD * errorRateRadPerSec) + (kS * Math.signum(errorRad));

        if (Math.abs(power) < LL_MIN_MOVE_POWER) {
            power = LL_MIN_MOVE_POWER * Math.signum(errorRad);
        }

        return Range.clip(power, -LL_MAX_POWER, LL_MAX_POWER);
    }

    private double derivative(double errorRad) {
        long now = System.nanoTime();
        double dt;

        if (lastTimeNs == 0) {
            dt = 0.02;
        } else {
            dt = (now - lastTimeNs) / 1e9;
        }

        lastTimeNs = now;

        if (dt <= 0) dt = 0.02;

        double rate = (errorRad - lastErrorRad) / dt;
        lastErrorRad = errorRad;
        return rate;
    }

    private void resetDerivative() {
        lastErrorRad = 0.0;
        lastTimeNs = 0;
    }

    private double getTurretYawRad() {
        YawPitchRollAngles angles = turretImu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }

    private static double wrapAngle(double rad) {
        while (rad <= -Math.PI) rad += 2.0 * Math.PI;
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        return rad;
    }

    private void startLimelight() {
        if (limelightOn) return;

        limelight.setPollRateHz(pollRateHz);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        limelightOn = true;
        lastTargetSeenNs = System.nanoTime();
    }

    private void stopLimelight() {
        if (!limelightOn) return;
        limelight.stop();
        limelightOn = false;
    }
}
