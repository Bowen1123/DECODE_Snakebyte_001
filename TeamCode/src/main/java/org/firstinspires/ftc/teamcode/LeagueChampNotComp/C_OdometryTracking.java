package org.firstinspires.ftc.teamcode.LeagueChampNotComp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Auto-aim turret using odometry (x,y) to compute target field angle to a goal point,
 * and a turret-mounted REV 9-axis IMU to measure turret field yaw.
 *
 * - goalX/goalY fixed to 48,48 (Dashboard adjustable if you want)
 * - initPos unchanged (0,0,0)
 * - turret powered by CRServo
 * - dynamic kS + minimum movement power outside deadband to prevent stall heating
 */
@Config
@Disabled
@TeleOp
public class C_OdometryTracking extends LinearOpMode {

    // ===== Goal position (field coordinates, SAME units as your odometry pose) =====
    public static double goalX = 51.0;
    public static double goalY = 41.0;

    // ===== Turret control (Dashboard) =====
    public static double kP = 1.3;
    public static double kD = 0.05;

    // Dynamic static assist
    public static double kS_min = 0.02;
    public static double kS_max = 0.12;
    public static double kS_fullAtDeg = 12.0;

    // Deadband + clamps
    public static double deadbandDeg = 1.0;
    public static double MIN_MOVE_POWER = 0.17;
    public static double MAX_POWER = 0.60;

    public static boolean invertTurret = false;

    // Optional yaw offset (radians) if your IMU zero isn't exactly "turret forward"
    public static double turretOffsetRad = 0.0;
    private double lastError = 0.0;
    private long lastTimeNs = 0;

    // Hardware
    private CRServo turret;
    private IMU turretImu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init pos unchanged
        Pose2d initPos = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPos);

        turret = hardwareMap.get(CRServo.class, "turret");
        turretImu = hardwareMap.get(IMU.class, "turretImu"); // set this name in Robot Config

        // Define turret yaw = 0 at init (make sure turret is physically at your "zero" direction)
        turretImu.resetYaw();

        waitForStart();

        while (opModeIsActive()) {
            // Update localization
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            double robotX = pose.position.x;
            double robotY = pose.position.y;

            // Target field angle from robot to goal point
            double targetFieldAngle = Math.atan2(goalY - robotY, goalX - robotX);

            // Turret's current field yaw from turret IMU (+ optional offset)
            double turretFieldYaw = getTurretYawRad() + turretOffsetRad;

            // Error = where we want turret to point in field frame - where it is pointing now
            double error = angleWrap(targetFieldAngle - turretFieldYaw);

            long now = System.nanoTime();
            double dt = (lastTimeNs == 0) ? 0.02 : (now - lastTimeNs) / 1e9;
            lastTimeNs = now;
            if (dt <= 0) dt = 0.02;

            double dErr = (error - lastError) / dt;
            lastError = error;

            // ===== Command with dynamic kS + guaranteed minimum power outside deadband =====
            double deadbandRad = Math.toRadians(deadbandDeg);
            double cmd;
            double absErr = Math.abs(error);

            if (absErr <= deadbandRad) {
                cmd = 0.0; // stop inside deadband (prevents heating)
            } else {
                // Dynamic kS ramp based on error magnitude
                double fullAtRad = Math.toRadians(kS_fullAtDeg);
                double t = Range.clip(absErr / fullAtRad, 0.0, 1.0);
                double smooth = (3.0 * t * t) - (2.0 * t * t * t);
                double kS_dyn = kS_min + (kS_max - kS_min) * smooth;

                // P + dynamic static assist
                cmd = (kP * error) + (kD * dErr) + (kS_dyn * Math.signum(error));

                // Enforce minimum movement power (your requirement)
                if (Math.abs(cmd) < MIN_MOVE_POWER) {
                    cmd = MIN_MOVE_POWER * Math.signum(error);
                }

                // Clamp
                cmd = Range.clip(cmd, -MAX_POWER, MAX_POWER);
            }

            if (invertTurret) cmd *= -1.0;

            turret.setPower(cmd);

            // Telemetry
            telemetry.addData("Robot (x,y)", "(%.2f, %.2f)", robotX, robotY);
            telemetry.addData("Goal (x,y)", "(%.2f, %.2f)", goalX, goalY);
            telemetry.addData("TargetFieldDeg", "%.2f", Math.toDegrees(targetFieldAngle));
            telemetry.addData("TurretFieldDeg", "%.2f", Math.toDegrees(turretFieldYaw));
            telemetry.addData("ErrorDeg", "%.2f", Math.toDegrees(error));
            telemetry.addData("Cmd", "%.3f", cmd);
            telemetry.update();


            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            Range.clip(-gamepad1.left_stick_y, -0.6, 0.6),
                            Range.clip(-gamepad1.left_stick_x, -0.6, 0.6)
                    ),
                    Range.clip(-gamepad1.right_stick_x, -0.6, 0.6)
            ));
        }
    }

    private double getTurretYawRad() {
        YawPitchRollAngles ypr = turretImu.getRobotYawPitchRollAngles();
        return ypr.getYaw(AngleUnit.RADIANS);
    }

    /** Wrap angle to (-pi, pi] */
    private static double angleWrap(double rad) {
        while (rad <= -Math.PI) rad += 2.0 * Math.PI;
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        return rad;
    }
}