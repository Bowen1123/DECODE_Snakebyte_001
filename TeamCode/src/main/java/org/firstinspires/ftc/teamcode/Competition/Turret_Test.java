package org.firstinspires.ftc.teamcode.Competition;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Turret_Test", group = "Test")
public class Turret_Test extends LinearOpMode {

    private Limelight3A limelight;
    private CRServo turret;

    // ---- TUNING ----
    // tx is in degrees. kP converts degrees -> CRServo power.
    private static final double kP = 0.02;

    // Prevents jitter when close to centered
    private static final double DEADBAND_DEG = 1.0;

    // Safety cap so the turret doesn't whip around
    private static final double MAX_POWER = 0.75;

    // If your turret moves the wrong way, set this to -1
    private static final double DIRECTION = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret = hardwareMap.get(CRServo.class, "turret");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            // Status is nice for debugging
            LLStatus status = limelight.getStatus();
            telemetry.addData("LL", "Temp: %.1fC | CPU: %.1f%% | FPS: %d | Pipeline: %d (%s)",
                    status.getTemp(), status.getCpu(), (int) status.getFps(),
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();

            boolean hasTarget = (result != null && result.isValid());

            double tx = 0.0; // horizontal offset (deg)
            double ty = 0.0; // vertical offset (deg)

            if (hasTarget) {
                tx = result.getTx();
                ty = result.getTy();
            }

            // ---- Control output for a continuous servo turret ----
            // Goal: drive tx toward 0
            double turretPowerCmd = 0.0;

            if (hasTarget) {
                if (Math.abs(tx) > DEADBAND_DEG) {
                    turretPowerCmd = DIRECTION * (kP * tx);
                    turretPowerCmd = clamp(turretPowerCmd, -MAX_POWER, MAX_POWER);
                } else {
                    turretPowerCmd = 0.0;
                }
            } else {
                turretPowerCmd = 0.0; // no target -> stop
            }

            // Apply command to turret
            turret.setPower(turretPowerCmd);

            // ---- Telemetry ----
            telemetry.addData("Has Target", hasTarget);
            telemetry.addData("tx (deg)", tx);
            telemetry.addData("ty (deg)", ty);
            telemetry.addData("Turret Power Cmd", "%.3f", turretPowerCmd);

            if (hasTarget) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("Botpose", botpose != null ? botpose.toString() : "null");

            } else {
                telemetry.addData("Limelight", "No valid result / no target");
            }

            telemetry.addLine("Tip: If turret turns away from target, set DIRECTION = -1.0");
            telemetry.update();
        }

        // Stop cleanly
        turret.setPower(0.0);
        limelight.stop();
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}