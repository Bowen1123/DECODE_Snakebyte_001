package org.firstinspires.ftc.teamcode.A_Regional;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Component testing for TRANSFER + SHOOTER + TURRET (Limelight AprilTag tracking)
 *
 * TRANSFER (Gamepad1):
 *  - Right Trigger : outtake()
 *  - Right Bumper  : intake()
 *  - Left Bumper   : properOuttake()
 *  - else          : deactivate()
 *
 * SHOOTER (Gamepad1):
 *  - X toggle shooter enable
 *
 * TURRET (Gamepad1):
 *  - A toggle AprilTag tracking (uses Limelight tx)
 *
 * Distance:
 *  - Uses Limelight ty + mount geometry inside shooterTurret.getGroundDistanceInches()
 *  - If no valid target, keep last distanceIn
 */
@Config
@TeleOp(group = "Component")
public class C_ComponentTesting extends LinearOpMode {
    /** Intake **/
    private DcMotorEx intakeMotor;
    private C_Intake intake;

    /** TRANSFER **/
    private DcMotorEx transferMotor;
    private Servo transferGate;
    private RevColorSensorV3 color;
    private C_Transfer transfer;

    /** SHOOTER + TURRET (COMBINED) **/
    private C_ShooterTurret shooterTurret;

    /** Drive **/
    private MecanumDrive drive;

    // Toggle state
    private boolean turretTracking = false;
    private boolean shooterEnabled = false;

    private double distanceIn = 70;

    /// Drive
    private double dt_max_power = 0.67;

    private double targetRpm = 2400, targetRampPos = 0.38;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize_shooterTurret();
        initialize_intake();
        initialize_transfer();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // If you want to use test targets (currently commented out in shooterUpdate() in C_ShooterTurret)
        shooterTurret.setTestTargetRampPos(targetRampPos);
        shooterTurret.setTestTargetRPM(targetRpm);

        waitForStart();

        while (opModeIsActive()) {

            // ---------------- TRANSFER + INTAKE ----------------
            if (gamepad1.right_trigger > .2) {
                intake.outtake();
                transfer.outtake();
            } else if (gamepad1.right_bumper) {
                intake.intake();
                transfer.intake();
            } else if (gamepad1.left_bumper) {
                transfer.properOuttake();
                intake.properOuttake();
            } else {
                intake.deactivate();
                transfer.deactivate();
            }

            // ---------------- TURRET TRACKING TOGGLE ----------------
            if (gamepad1.aWasPressed()) {
                turretTracking = !turretTracking;
                shooterTurret.setTrackingEnabled(turretTracking);
            }

            ///  ADDED THIS MORNING
//            if (!shooterTurret.isTrackingEnabled()){
//                if (gamepad2.left_bumper){
//                    shooterTurret.setTurretPower(.25);
//                } else if (gamepad2.right_bumper){
//                    shooterTurret.setTurretPower(-.25);
//                } else {
//                    shooterTurret.setTurretPower(0);
//                }
//            }



            // Update Limelight + run turret loop
            shooterTurret.updateLimelight();
            boolean aimed = shooterTurret.turretLoop();


            // ---------------- DISTANCE SOURCE (Limelight if valid) ----------------
            if (shooterTurret.hasTarget()) {
                distanceIn = shooterTurret.getGroundDistanceInches();
            }

            // ---------------- SHOOTER TOGGLE ----------------
            if (gamepad1.xWasPressed()) {
                shooterEnabled = !shooterEnabled;
                shooterTurret.setShooterEnabled(shooterEnabled);
                shooterTurret.resetShooterController();
            }

            // Shooter update
            shooterTurret.syncFromDashboard();
            shooterTurret.setDistanceInches(distanceIn);
            double shooterPowerCmd = shooterTurret.shooterUpdate();

            ///  Drive

            if (gamepad2.yWasPressed()){
                dt_max_power = 0.9;
            } else if (gamepad2.xWasPressed()){
                dt_max_power = 0.7;
            } else if (gamepad2.aWasPressed()){
                dt_max_power = 0.5;
            }

            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            double forwardMax = dt_max_power;
            double strafeMax = Math.min(dt_max_power * 1.2, 1);

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            Range.clip(Math.pow(-gamepad1.left_stick_y, 3), -forwardMax, forwardMax),
                            Range.clip(Math.pow(-gamepad1.left_stick_x, 3), -strafeMax, strafeMax)
                    ),
                    Range.clip(Math.pow(-gamepad1.right_stick_x, 3), -forwardMax, forwardMax)
            ));

            // Pipeline switching (uses shooterTurret now)
            if (gamepad2.yWasPressed()) {
                shooterTurret.setPipeline(1);
            }
            if (gamepad2.xWasPressed()) {
                shooterTurret.setPipeline(0);
            }

            if (!shooterTurret.isTrackingEnabled()) {
                shooterTurret.setBothLEDPos(0.71);
            } else if (aimed){
                shooterTurret.setBothLEDPos(0.5);
            } else {
                shooterTurret.setBothLEDPos(0.3);
            }

            telemetry.addData("Pipeline", shooterTurret.getPipeline());
            telemetry.addData("TEST RPM", targetRpm);
            telemetry.addData("TEST POS", targetRampPos);

            // ---------------- TELEMETRY ----------------
            telemetry.addLine("=== TRANSFER ===");
            telemetry.addData("Gate Pos", transferGate.getPosition());
            telemetry.addData("Transfer Power", transferMotor.getPower());

            telemetry.addLine("=== TURRET / LIMELIGHT ===");
            telemetry.addData("Tracking Enabled", shooterTurret.isTrackingEnabled());
            telemetry.addData("Aimed (deadband)", aimed);
            telemetry.addData("Has Target", shooterTurret.hasTarget());
            telemetry.addData("tx (deg)", shooterTurret.getTxDeg());
            telemetry.addData("ty (deg)", shooterTurret.getTyDeg());
            telemetry.addData("Distance (in)", distanceIn);

            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Enabled", shooterTurret.isShooterEnabled());
            telemetry.addData("Target RPM", "%.0f", shooterTurret.getTargetRPM());
            telemetry.addData("Measured RPM", "%.0f", shooterTurret.getMeasuredRPM());
            telemetry.addData("Power Cmd", "%.3f", shooterPowerCmd);
            telemetry.addData("Ramp Pos", "%.3f", shooterTurret.getRampPosition());
            telemetry.addData("At Target", shooterTurret.isAtTarget());

            telemetry.update();
        }
    }

    private void initialize_intake() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intake = new C_Intake(intakeMotor);
    }

    private void initialize_transfer() {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        transferGate = hardwareMap.get(Servo.class, "transferGate");

        // color is unused; keep null-safe
        try {
            color = hardwareMap.get(RevColorSensorV3.class, "color");
        } catch (Exception e) {
            color = null;
        }

        transfer = new C_Transfer(transferMotor, transferGate, color);
    }

    private void initialize_shooterTurret() {
        // Shooter hardware
        DcMotorEx topShooter = hardwareMap.get(DcMotorEx.class, "topShooter");
        DcMotorEx bottomShooter = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        Servo shooterRamp = hardwareMap.get(Servo.class, "shooterRamp");
        Servo leftLED = hardwareMap.get(Servo.class, "leftLED");
        Servo rightLED = hardwareMap.get(Servo.class, "rightLED");

        // Turret hardware
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        CRServo turretServo = hardwareMap.get(CRServo.class, "turret");

        // Combined class (does all old init/start internally)
        shooterTurret = new C_ShooterTurret(
                topShooter, bottomShooter, shooterRamp, leftLED, rightLED,
                limelight, turretServo
        );

        shooterTurret.setShooterEnabled(false);
        shooterTurret.setTrackingEnabled(false);
    }
}