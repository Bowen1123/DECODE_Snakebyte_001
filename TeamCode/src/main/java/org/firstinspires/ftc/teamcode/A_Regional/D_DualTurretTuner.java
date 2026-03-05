package org.firstinspires.ftc.teamcode.A_Regional;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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

/**
 * D_DualTurretTuner (IMU + dual-stage, tuned like BasicTurret)
 *
 * Buttons:
 *  - A: toggle TrackingEnabled
 *  - B: LOCK_TO_DRIVE
 *  - X: FORCE ODOM_ONLY
 *  - Y: FORCE LL_ONLY
 *  - Dpad: nudge POI (goalX/goalY) to validate odom math
 *  - Back: sync turret IMU heading to drivetrain heading (offset update)
 *  - Start: reset turret IMU yaw and sync to drivetrain
 */
@Config
@Disabled
@TeleOp(group = "Tuning")
public class D_DualTurretTuner extends LinearOpMode {

    public static double DT_MAX_POWER = 0.6;

    public static String LIMELIGHT_NAME = "limelight";
    public static String TURRET_SERVO_NAME = "turret";
    public static String DRIVE_IMU_NAME = "imu";
    public static String TURRET_IMU_NAME = "turretImu";

    public static double POI_STEP = 2.0;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Limelight3A ll = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        CRServo turretServo = hardwareMap.get(CRServo.class, TURRET_SERVO_NAME);

        IMU driveImu = hardwareMap.get(IMU.class, DRIVE_IMU_NAME);
        IMU turretImu = hardwareMap.get(IMU.class, TURRET_IMU_NAME);

        driveImu.resetYaw();
        turretImu.resetYaw();

        D_DualTurret turret = new D_DualTurret(ll, turretImu);
        turret.init(turretServo);
        turret.start();
        turret.setTrackingEnabled(false);
        turret.setMode(D_DualTurret.Mode.AUTO_DUAL);

        boolean lastA=false, lastB=false, lastX=false, lastY=false, lastBack=false, lastStart=false;

        waitForStart();

        while (opModeIsActive()) {

            // Drive
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            Range.clip(Math.pow(-gamepad1.left_stick_y, 3), -DT_MAX_POWER, DT_MAX_POWER),
                            Range.clip(Math.pow(-gamepad1.left_stick_x, 3), -DT_MAX_POWER, DT_MAX_POWER)
                    ),
                    Range.clip(Math.pow(-gamepad1.right_stick_x, 3), -DT_MAX_POWER, DT_MAX_POWER)
            ));

            // Drivetrain yaw
            YawPitchRollAngles a = driveImu.getRobotYawPitchRollAngles();
            double driveYawRad = a.getYaw(AngleUnit.RADIANS);

            // Mode buttons
            boolean A = gamepad1.a;
            boolean B = gamepad1.b;
            boolean X = gamepad1.x;
            boolean Y = gamepad1.y;
            boolean BACK = gamepad1.back;
            boolean START = gamepad1.start;

            if (A && !lastA) turret.setTrackingEnabled(!turret.isTrackingEnabled());
            if (B && !lastB) turret.setMode(D_DualTurret.Mode.LOCK_TO_DRIVE);
            if (X && !lastX) turret.setMode(D_DualTurret.Mode.ODOM_ONLY);
            if (Y && !lastY) turret.setMode(D_DualTurret.Mode.LL_ONLY);

            // IMU sync helpers
            if (BACK && !lastBack) turret.syncTurretImuToDrivetrain(driveYawRad);
            if (START && !lastStart) turret.resetTurretImuYawAndSync(driveYawRad);

            lastA=A; lastB=B; lastX=X; lastY=Y; lastBack=BACK; lastStart=START;

            // POI nudge (lets you see odom response)
            if (gamepad1.dpad_up)    D_DualTurret.goalY += POI_STEP;
            if (gamepad1.dpad_down)  D_DualTurret.goalY -= POI_STEP;
            if (gamepad1.dpad_right) D_DualTurret.goalX += POI_STEP;
            if (gamepad1.dpad_left)  D_DualTurret.goalX -= POI_STEP;

            // Turret updates
            turret.updateLimelight();
            boolean aimed = turret.loop(pose, driveYawRad);

            telemetry.addLine("=== DualTurret (IMU) Tuner ===");
            telemetry.addData("TrackingEnabled", turret.isTrackingEnabled());
            telemetry.addData("Mode", turret.getMode());
            telemetry.addData("Stage", turret.getStage());
            telemetry.addData("Aimed", aimed);

            telemetry.addLine("=== POI ===");
            telemetry.addData("goalX", "%.2f", D_DualTurret.goalX);
            telemetry.addData("goalY", "%.2f", D_DualTurret.goalY);

            telemetry.addLine("=== Headings ===");
            telemetry.addData("DriveYawDeg", "%.2f", Math.toDegrees(driveYawRad));
            telemetry.addData("TurretAbsDeg", "%.2f", turret.getTurretAbsDeg());
            telemetry.addData("TurretRelDeg", "%.2f", turret.getTurretRelDeg(driveYawRad));

            telemetry.addLine("=== Errors / Output ===");
            telemetry.addData("ErrDeg(filt)", "%.2f", turret.getLastErrDeg());
            telemetry.addData("ErrRateDeg/s(filt)", "%.2f", turret.getLastErrRateDegPerSec());
            telemetry.addData("Out", "%.3f", turret.getLastOut());
            telemetry.addData("OdomErrDeg(raw)", "%.2f", turret.getLastOdomErrDeg());
            telemetry.addData("LLErrDeg(raw)", "%.2f", turret.getLastLLErrDeg());

            telemetry.addLine("=== Limelight ===");
            telemetry.addData("HasTarget", turret.hasTarget());
            telemetry.addData("tx", "%.2f", turret.getTxDeg());
            telemetry.addData("pipe", D_DualTurret.PIPELINE_INDEX);

            telemetry.addLine("Buttons: A=toggle, B=LOCK, X=ODOM_ONLY, Y=LL_ONLY, Back=sync, Start=reset+sync");
            telemetry.update();
        }
    }
}