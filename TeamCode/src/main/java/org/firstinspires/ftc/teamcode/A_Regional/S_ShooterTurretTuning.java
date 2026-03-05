package org.firstinspires.ftc.teamcode.A_Regional;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * S_ShooterTurretTuning
 *
 * Replaces the old S_CloseShooterPID_Slew-based tuner.
 * This OpMode tunes values in C_ShooterTurret via FTC Dashboard (@Config in that class).
 *
 * Gamepad1:
 *  - X : toggle shooter enable
 *  - A : toggle turret tracking enable
 *  - dpad_up/down : +/- 100 RPM (MANUAL_TARGET_RPM) when USE_MANUAL_SHOOTER_TARGET = true
 *  - Y : pipeline 1
 *  - B : pipeline 0
 *
 * Intake/Transfer passthrough (same style as your old tuner):
 *  - RT : intake + transfer (fast feed)
 *  - RB : intake + transfer (slower feed)
 *  - LB : reverse both
 */
@TeleOp(name = "S_ShooterTurretTuning", group = "Tuning")
public class S_ShooterTurretTuning extends OpMode {

    // Hardware for intake/transfer passthrough
    private DcMotorEx intake, transfer;

    // Combined mechanism
    private C_ShooterTurret shooterTurret;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // Toggle states
    private boolean shooterEnabled = false;
    private boolean turretTracking = false;

    // edge detect
    private boolean lastX = false;
    private boolean lastA = false;
    private boolean lastY = false;
    private boolean lastB = false;
    private boolean lastUp = false;
    private boolean lastDown = false;

    // same style constants as your old tuner
    private final double transfer_power = .9, intake_power = .8;
    private Servo transferGate;
    private double shooterRampPos = 0.38;


    @Override
    public void init() {
        // Intake/transfer motors (unchanged)
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        // ShooterTurret hardware
        DcMotorEx topShooter = hardwareMap.get(DcMotorEx.class, "topShooter");
        DcMotorEx bottomShooter = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        Servo shooterRamp = hardwareMap.get(Servo.class, "shooterRamp");
        Servo leftLED = hardwareMap.get(Servo.class, "leftLED");
        Servo rightLED = hardwareMap.get(Servo.class, "rightLED");

        transferGate = hardwareMap.get(Servo.class, "transferGate");

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        CRServo turretServo = hardwareMap.get(CRServo.class, "turret");

        shooterTurret = new C_ShooterTurret(
                topShooter, bottomShooter, shooterRamp, leftLED, rightLED,
                limelight, turretServo
        );

        shooterTurret.setShooterEnabled(false);
        shooterTurret.setTrackingEnabled(false);

        telemetry.addLine("S_ShooterTurretTuning ready");
        telemetry.addLine("Tune @Config in FTC Dashboard under: C_ShooterTurret");
        telemetry.addLine("Recommended: set C_ShooterTurret.USE_MANUAL_SHOOTER_TARGET = true");
        telemetry.addLine("X toggle shooter | A toggle turret | dpad up/down manual RPM | B/Y pipeline");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ---------------- Intake / Transfer passthrough ----------------
        if (gamepad1.right_trigger > 0.2) {
            intake.setPower(intake_power);
            transfer.setPower(transfer_power);
            transferGate.setPosition(0.65);
        } else if (gamepad1.right_bumper) {
            intake.setPower(intake_power);
            transfer.setPower(transfer_power * .6);
            transferGate.setPosition(0.58);

        } else if (gamepad1.left_bumper) {
            intake.setPower(-intake_power);
            transfer.setPower(-transfer_power);
            transferGate.setPosition(0.58);

        } else {
            intake.setPower(0);
            transfer.setPower(0);
            transferGate.setPosition(0.58);
        }

        // ---------------- Edge detect ----------------
        boolean x = gamepad1.x;
        boolean a = gamepad1.a;
        boolean y = gamepad1.y;
        boolean b = gamepad1.b;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        boolean xPressed = x && !lastX;
        boolean aPressed = a && !lastA;
        boolean yPressed = y && !lastY;
        boolean bPressed = b && !lastB;
        boolean upPressed = up && !lastUp;
        boolean downPressed = down && !lastDown;

        lastX = x; lastA = a; lastY = y; lastB = b; lastUp = up; lastDown = down;

        // ---------------- Pipeline control ----------------
        if (yPressed) shooterTurret.setPipeline(1);
        if (bPressed) shooterTurret.setPipeline(0);

        // ---------------- Shooter toggle ----------------
        if (xPressed) {
            shooterEnabled = !shooterEnabled;
            shooterTurret.setShooterEnabled(shooterEnabled);
            shooterTurret.resetShooterController();
        }

        // ---------------- Turret toggle ----------------
        if (aPressed) {
            turretTracking = !turretTracking;
            shooterTurret.setTrackingEnabled(turretTracking);
        }

        // ---------------- Manual RPM stepping (Dashboard source of truth) ----------------
        if (C_ShooterTurret.USE_MANUAL_SHOOTER_TARGET) {
            if (upPressed) C_ShooterTurret.MANUAL_TARGET_RPM += 100.0;
            if (downPressed) C_ShooterTurret.MANUAL_TARGET_RPM -= 100.0;

            // clamp manual rpm to sane bounds
            C_ShooterTurret.MANUAL_TARGET_RPM = Range.clip(
                    C_ShooterTurret.MANUAL_TARGET_RPM,
                    0,
                    C_ShooterTurret.CFG_maxTargetFlywheelRPM
            );
        }

        // ---------------- Turret update ----------------
        shooterTurret.updateLimelight();
        boolean aimed = false;
        if (shooterTurret.isTrackingEnabled()) {
            aimed = shooterTurret.turretLoop();
        }

        if (aimed){
            shooterTurret.setBothLEDPos(0.5);
        } else {
            shooterTurret.setBothLEDPos(0.3);
        }

        if (gamepad1.dpadLeftWasPressed()){
            shooterRampPos += 0.02;
        }
        if (gamepad1.dpadRightWasPressed()){
            shooterRampPos -= 0.02;
        }

        shooterTurret.setManualRampPos(shooterRampPos)
         ;

        // ---------------- Shooter update (PID runs here) ----------------
        shooterTurret.syncFromDashboard();
        double powerCmd = shooterTurret.shooterUpdate();

        // ---------------- PID error telemetry ----------------
        double targetRPM = shooterTurret.getTargetRPM();
        double measuredRPM = shooterTurret.getMeasuredRPM();
        double errorRPM = targetRPM - measuredRPM;

        telemetry.addLine("=== SHOOTER (C_ShooterTurret) ===");
        telemetry.addData("Enabled", shooterTurret.isShooterEnabled());
        telemetry.addData("ManualMode", C_ShooterTurret.USE_MANUAL_SHOOTER_TARGET);
        telemetry.addData("MANUAL_TARGET_RPM", "%.0f", C_ShooterTurret.MANUAL_TARGET_RPM);
        telemetry.addData("MANUAL_RAMP_POS", "%.3f", C_ShooterTurret.MANUAL_RAMP_POS);

        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Ramp Pos: ", shooterRampPos);
        telemetry.addData("Measured RPM", "%.0f", measuredRPM);
        telemetry.addData("Error RPM", "%.0f", errorRPM);
        telemetry.addData("Power Cmd", "%.3f", powerCmd);
        telemetry.addData("Ramp Pos", "%.3f", shooterTurret.getRampPosition());
        telemetry.addData("At Target", shooterTurret.isAtTarget());

        telemetry.addLine("=== TURRET / LIMELIGHT (C_ShooterTurret) ===");
        telemetry.addData("Tracking", shooterTurret.isTrackingEnabled());
        telemetry.addData("Turret Cmd: ", shooterTurret.getTurretCmd());
        telemetry.addData("Aimed", aimed);
        telemetry.addData("Has Target", shooterTurret.hasTarget());
        telemetry.addData("tx", "%.2f", shooterTurret.getTxDeg());
        telemetry.addData("ty", "%.2f", shooterTurret.getTyDeg());
        telemetry.addData("Distance(in)", "%.1f", shooterTurret.getGroundDistanceInches());
        telemetry.addData("Pipeline", shooterTurret.getPipeline());

        telemetry.update();

        // ---------------- Dashboard graphs ----------------
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("targetRPM", targetRPM);
        packet.put("currentRPM", measuredRPM);
        packet.put("errorRPM", errorRPM);
        packet.put("powerCmd", powerCmd);
        packet.put("shooterEnabled", shooterTurret.isShooterEnabled() ? 1 : 0);
        packet.put("turretTracking", shooterTurret.isTrackingEnabled() ? 1 : 0);
        packet.put("txDeg", shooterTurret.getTxDeg());
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        if (shooterTurret != null) {
            shooterTurret.stopAll();
        }
        if (intake != null) intake.setPower(0);
        if (transfer != null) transfer.setPower(0);
    }
}