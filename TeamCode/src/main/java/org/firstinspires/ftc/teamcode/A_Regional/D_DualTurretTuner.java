package org.firstinspires.ftc.teamcode.A_Regional;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(group = "Turret")
public class D_DualTurretTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "limelight");
        CRServo turretServo = hardwareMap.get(CRServo.class, "turret");
        DcMotorEx turretEncoder = hardwareMap.get(DcMotorEx.class, "topShooter");

        // ---------------- YOUR RR DRIVE HERE ----------------
        // Replace with your Pinpoint + RR mecanum drive.
        // Must provide pose heading (radians). We'll convert to degrees.
        Object drive = null; // placeholder
        // ----------------------------------------------------

        D_DualTurret turret = new D_DualTurret(ll);
        turret.init(turretServo, turretEncoder);
        turret.start();

        // Start disabled, pick mode with A/B/X/Y to enable
        turret.setTrackingEnabled(false);
        turret.setMode(D_DualTurret.Mode.DUAL_STAGE);

        FtcDashboard dash = FtcDashboard.getInstance();

        waitForStart();

        boolean lastA=false, lastB=false, lastX=false, lastY=false;
        boolean lastBack=false, lastStart=false;

        while (opModeIsActive()) {

            // ---- Update RR pose (REPLACE with your real drive code) ----
            // Example:
            // drive.updatePoseEstimate();
            // Pose2d pose = drive.localizer.getPose();
            Pose2d pose = new Pose2d(0, 0, 0); // placeholder
            // -----------------------------------------------------------

            double drivetrainHeadingDeg = Math.toDegrees(pose.heading.toDouble());

            // Buttons set modes directly:
            // A -> Limelight only
            // B -> Lock to drivetrain
            // X -> Odometry only
            // Y -> Dual stage
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;

            // Optional:
            // BACK -> disable tracking immediately
            // START -> enable tracking (without changing mode)
            boolean back = gamepad1.back;
            boolean start = gamepad1.start;

            if (a && !lastA) {
                turret.setMode(D_DualTurret.Mode.LIMELIGHT_ONLY);
                turret.setTrackingEnabled(true);
            }
            if (b && !lastB) {
                turret.setMode(D_DualTurret.Mode.LOCK_TO_DRIVETRAIN);
                turret.setTrackingEnabled(true);
            }
            if (x && !lastX) {
                turret.setMode(D_DualTurret.Mode.ODOMETRY_ONLY);
                turret.setTrackingEnabled(true);
            }
            if (y && !lastY) {
                turret.setMode(D_DualTurret.Mode.DUAL_STAGE);
                turret.setTrackingEnabled(true);
            }

            if (back && !lastBack) turret.setTrackingEnabled(false);
            if (start && !lastStart) turret.setTrackingEnabled(true);

            lastA=a; lastB=b; lastX=x; lastY=y;
            lastBack=back; lastStart=start;

            boolean aimed = turret.update(drivetrainHeadingDeg);

            // ---- Tuning-friendly telemetry (degrees) ----
            double turretRelDeg = turret.getTurretRelativeHeadingDeg();

            double targetRelDeg;
            if (turret.getMode() == D_DualTurret.Mode.LIMELIGHT_ONLY) {
                targetRelDeg = Double.NaN;
            } else if (turret.getMode() == D_DualTurret.Mode.LOCK_TO_DRIVETRAIN) {
                targetRelDeg = 0.0;
            } else {
                targetRelDeg = D_DualTurret.wrapDeg(D_DualTurret.GOAL_FIELD_HEADING_DEG - drivetrainHeadingDeg);
            }

            double odomErrDeg = Double.isNaN(targetRelDeg) ? Double.NaN : D_DualTurret.wrapDeg(targetRelDeg - turretRelDeg);

            String stage;
            switch (turret.getMode()) {
                case LIMELIGHT_ONLY:
                    stage = "VISION";
                    break;
                case LOCK_TO_DRIVETRAIN:
                    stage = "ODOM(LOCK)";
                    break;
                case ODOMETRY_ONLY:
                    stage = "ODOM";
                    break;
                case DUAL_STAGE:
                default:
                    boolean gate = Math.abs(odomErrDeg) <= D_DualTurret.ODOM_TO_VISION_GATE_DEG;
                    stage = (gate && turret.hasTarget()) ? "VISION" : "ODOM";
                    break;
            }

            TelemetryPacket p = new TelemetryPacket();
            p.put("trackingEnabled", turret.isTrackingEnabled());
            p.put("mode", turret.getMode().toString());
            p.put("stage", stage);
            p.put("aimed", aimed);

            p.put("goalFieldDeg", D_DualTurret.GOAL_FIELD_HEADING_DEG);
            p.put("driveHeadingDeg", drivetrainHeadingDeg);

            p.put("turretRelDeg", turretRelDeg);
            p.put("targetRelDeg", targetRelDeg);
            p.put("odomErrDeg", odomErrDeg);
            p.put("gateDeg", D_DualTurret.ODOM_TO_VISION_GATE_DEG);

            p.put("LL_hasTarget", turret.hasTarget());
            p.put("tx_deg", turret.getTxDeg());
            p.put("ty_deg", turret.getTyDeg());

            dash.sendTelemetryPacket(p);

            telemetry.addData("Controls", "A=LL  B=Lock  X=Odom  Y=Dual  (Start=Enable, Back=Disable)");
            telemetry.addData("tracking", turret.isTrackingEnabled());
            telemetry.addData("mode", turret.getMode());
            telemetry.addData("stage", stage);
            telemetry.addData("aimed", aimed);

            telemetry.addData("goalFieldDeg", "%.1f", D_DualTurret.GOAL_FIELD_HEADING_DEG);
            telemetry.addData("driveHeadingDeg", "%.1f", drivetrainHeadingDeg);

            telemetry.addData("turretRelDeg", "%.1f", turretRelDeg);
            telemetry.addData("targetRelDeg", "%.1f", targetRelDeg);
            telemetry.addData("odomErrDeg", "%.1f", odomErrDeg);

            telemetry.addData("LL hasTarget", turret.hasTarget());
            telemetry.addData("LL tx", "%.2f", turret.getTxDeg());
            telemetry.update();
        }

        turret.setTrackingEnabled(false);
        turret.stop();
    }
}