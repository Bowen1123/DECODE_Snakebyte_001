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
 *  - Right Bumper  : intake()
 *  - Right Trigger : outtake()
 *  - else          : deactivate()
 *
 * SHOOTER (Gamepad2):
 *  - X toggle shooter enable
 *  - DpadUp/Down adjust TEST_DISTANCE_IN (fallback when no tag)
 *  - Y ramp max | A ramp min (sanity)
 *
 * TURRET (Gamepad2):
 *  - B toggle AprilTag tracking (uses Limelight tx)
 *  - If tracking enabled, turret auto aims using D_Turret.loop()
 *  - If tracking disabled, turret power is 0 (or you can add manual bumpers)
 *
 * Distance:
 *  - Uses Limelight ty + mount geometry inside D_Turret.getGroundDistanceInches()
 *  - If no valid target, shooter uses C_Shooter.TEST_DISTANCE_IN fallback
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

    /** SHOOTER **/
    private DcMotorEx topShooter, bottomShooter;
    private Servo shooterRamp;
    private C_Shooter shooter;
    private Servo leftLED, rightLED;

    /** TURRET + LIMELIGHT **/
    private Limelight3A limelight;
    private CRServo turretServo;
    private D_BasicTurret turret;

    /** Drive (optional, kept for structure) **/
    private MecanumDrive drive;

    // Toggle state
    private boolean lastB2 = false;
    private boolean turretTracking = false;

    private boolean lastX2 = false;
    private boolean shooterEnabled = false;

    private double  distanceIn = 70;
    /// Drive
    private double dt_max_power = 0.9;
    private double targetRpm = 2400, targetRampPos = 0.38;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize_transfer();
        initialize_shooter();
        initialize_turret();
        initialize_intake();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        shooter.setTestTargetRampPos(targetRampPos);
        shooter.setTestTargetRPM(targetRpm);

        waitForStart();

        while (opModeIsActive()) {

            // ---------------- TRANSFER ----------------
            if (gamepad1.right_trigger > 0.2) {
                intake.outtake();
                transfer.outtake();
            } else if (gamepad1.right_bumper) {
                intake.intake();
                transfer.intake();
            } else {
                intake.deactivate();
                transfer.deactivate();
            }

            // ---------------- TURRET TRACKING TOGGLE ----------------
            if (gamepad1.aWasPressed()) {
                turretTracking = !turretTracking;
                turret.setTrackingEnabled(turretTracking);
            }

            // Update Limelight + run turret loop
            turret.updateLimelight();
            boolean aimed = turret.loop();

            // ---------------- DISTANCE SOURCE (Limelight if valid, else fallback) ----------------
            if (turret.hasTarget()) {
                distanceIn = turret.getGroundDistanceInches();
            }

            // ---------------- SHOOTER TOGGLE ----------------
            if (gamepad1.xWasPressed()) {
                shooterEnabled = !shooterEnabled;
                shooter.setEnabled(shooterEnabled);
                shooter.resetController();
            }

            ;
            ///  TEST VALUES ///
//            if (gamepad1.dpadDownWasPressed()) {
//                targetRpm -= 50;
//            }
//            if (gamepad1.dpadUpWasPressed()) {
//                targetRpm += 50;
//            }
//
//            if (gamepad1.dpadRightWasPressed()) {
//                targetRampPos -= .02;
//            }
//            if (gamepad1.dpadLeftWasPressed()) {
//                targetRampPos += .02;
//            }
//            shooter.setTestTargetRampPos(targetRampPos);
//            shooter.setTestTargetRPM(targetRpm);

            // Shooter update
            shooter.syncFromDashboard();
            shooter.setDistanceInches(distanceIn);
            double shooterPowerCmd = shooter.update();

            

            ///  Drive
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            Range.clip(-gamepad1.left_stick_y, -dt_max_power, dt_max_power),
                            Range.clip(-gamepad1.left_stick_x, -dt_max_power, dt_max_power)
                    ),
                    Range.clip(-gamepad1.right_stick_x, -dt_max_power * .5, dt_max_power * .5)
            ));


            telemetry.addData("TEST RPM: ", targetRpm);
            telemetry.addData("TEST POS: ", targetRampPos);

            // ---------------- TELEMETRY ----------------
            telemetry.addLine("=== TRANSFER ===");
            telemetry.addData("Gate Pos", transferGate.getPosition());
            telemetry.addData("Transfer Power", transferMotor.getPower());

            telemetry.addLine("=== TURRET / LIMELIGHT ===");
            telemetry.addData("Tracking Enabled", turret.isTrackingEnabled());
            telemetry.addData("Aimed (deadband)", aimed);
            telemetry.addData("Has Target", turret.hasTarget());
            telemetry.addData("tx (deg)", turret.getTxDeg());
            telemetry.addData("ty (deg)", turret.getTyDeg());
            telemetry.addData("Distance (in)", distanceIn);

            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Enabled", shooter.isEnabled());
            telemetry.addData("Target RPM", "%.0f", shooter.getTargetRPM());
            telemetry.addData("Measured RPM", "%.0f", shooter.getMeasuredRPM());
            telemetry.addData("Power Cmd", "%.3f", shooterPowerCmd);
            telemetry.addData("Ramp Pos", "%.3f", shooter.getRampPosition());
            telemetry.addData("At Target", shooter.isAtTarget());

            telemetry.update();
        }
    }
    private void initialize_intake() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        intake = new C_Intake(intakeMotor);
    }

    private void initialize_transfer() {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        transferGate  = hardwareMap.get(Servo.class, "transferGate");
        color = hardwareMap.get(RevColorSensorV3.class, "color");

        // Keep this matching your current constructor usage:
        transfer = new C_Transfer(transferMotor, transferGate, color);
        transfer.deactivate();
    }

    private void initialize_shooter() {
        topShooter = hardwareMap.get(DcMotorEx.class, "topShooter");
        bottomShooter = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        shooterRamp = hardwareMap.get(Servo.class, "shooterRamp");

        leftLED = hardwareMap.get(Servo.class, "leftLED");
        rightLED = hardwareMap.get(Servo.class, "rightLED");

        shooter = new C_Shooter(topShooter, bottomShooter, shooterRamp, leftLED, rightLED);
        shooter.setEnabled(false);
    }

    private void initialize_turret() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(CRServo.class, "turret");

        turret = new D_BasicTurret(limelight);
        turret.init(turretServo);
        turret.start();
        turret.setTrackingEnabled(false);
    }

}