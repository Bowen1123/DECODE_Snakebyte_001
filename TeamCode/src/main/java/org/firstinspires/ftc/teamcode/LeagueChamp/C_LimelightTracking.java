package org.firstinspires.ftc.teamcode.LeagueChamp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@Config
@Disabled
@TeleOp
public class C_LimelightTracking extends LinearOpMode {

    // ===== Limelight config (Dashboard) =====
    public static int PIPELINE = 0;          // AprilTag pipeline index on Limelight
    public static int targetId = 20;         // -1 = auto pick best, else specific tag ID
    public static int pollRateHz = 100;      // LL polling rate

    // ===== Turret control using tx (Dashboard) =====
    // Control objective: tx -> 0 degrees
    public static double kP = 1.3;           // power per deg error
    public static double kD = 0.05;          // power per (deg/sec) error rate

    // Dynamic static assist
    public static double kS_min = 0.02;
    public static double kS_max = 0.12;
    public static double kS_fullAtDeg = 12.0;

    public static double deadbandDeg = 0.8;

    // Minimum movement power outside deadband (your requirement)
    public static double MIN_MOVE_POWER = 0.18;

    // Clamp
    public static double MAX_POWER = 0.60;

    public static boolean invertTurret = false;

    // ===== Distance telemetry =====
    public static boolean distanceInInches = true;
    private static final double M_TO_IN = 39.37007874;

    // ===== Proportions-based range config (Dashboard) =====
    // Option A (physics-ish): range from tag pixel height using VFOV + image height
    public static boolean useCornerBasedRange = true; // true = corners/VFOV model, false = area-based model

    public static double TAG_SIZE_M = 0.165;   // tag side length in meters (set to your field tag size)
    public static double CAM_VFOV_DEG = 49.7;  // Limelight vertical FOV (set to your model)
    public static int CAM_HEIGHT_PX = 720;     // pipeline image height (e.g., 720, 480, etc.)

    // Option B (empirical): range from target area ta -> range = AREA_K / sqrt(ta)
    public static double AREA_K = 1.2;         // calibrate: AREA_K = knownDistMeters * sqrt(ta)

    // Hardware
    private Limelight3A limelight;
    private CRServo turret;

    // Derivative state
    private double lastErrorDeg = 0.0;
    private long lastTimeNs = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret = hardwareMap.get(CRServo.class, "turret");

        limelight.setPollRateHz(pollRateHz);
        limelight.start();
        limelight.pipelineSwitch(PIPELINE);

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Horizontal offset to tag center
                double txDeg = result.getTx();

                // We want tx -> 0, so error is negative of tx
                double errorDeg = -txDeg;

                // Compute derivative (deg/sec)
                long now = System.nanoTime();
                double dt = (lastTimeNs == 0) ? 0.02 : (now - lastTimeNs) / 1e9;
                lastTimeNs = now;
                if (dt <= 0) dt = 0.02;

                double dErrorDeg = (errorDeg - lastErrorDeg) / dt;
                lastErrorDeg = errorDeg;

                // Turret command with P + D + dynamic kS + min power
                double cmd = computeTurretCmdDeg(errorDeg, dErrorDeg);
                if (invertTurret) cmd *= -1.0;
                turret.setPower(cmd);

                // Proportions-based distance robot -> tag (meters)
                Double rangeM = getRangeToTagByProportions(result);

                telemetry.addData("tx (deg)", txDeg);
                telemetry.addData("error (deg)", errorDeg);
                telemetry.addData("dError (deg/s)", dErrorDeg);
                telemetry.addData("cmd", cmd);

                if (rangeM != null) {
                    telemetry.addData("Range", distanceInInches ? (rangeM * M_TO_IN) : rangeM);
                    telemetry.addData("Units", distanceInInches ? "in" : "m");
                    telemetry.addData("RangeMode", useCornerBasedRange ? "Corners(VFOV)" : "Area(ta)");
                } else {
                    telemetry.addData("Range", "No tag proportions");
                }

                telemetry.addData("Pipeline", result.getPipelineIndex());
                telemetry.addData("Staleness (ms)", result.getStaleness());
            } else {
                // No target -> stop turret, and reset derivative timing so it doesn't spike next time
                turret.setPower(0.0);
                lastTimeNs = 0;
                lastErrorDeg = 0.0;
                telemetry.addData("Limelight", "No Targets");
            }

            telemetry.update();
        }
    }

    /**
     * P + D + dynamic kS + enforced minimum power when outside deadband.
     * errorDeg: desired correction in degrees (positive means rotate turret + direction).
     * dErrorDeg: error rate in deg/sec.
     */
    private double computeTurretCmdDeg(double errorDeg, double dErrorDeg) {
        double absErr = Math.abs(errorDeg);

        if (absErr <= deadbandDeg) {
            return 0.0; // inside deadband: no power (prevents heating)
        }

        // Dynamic kS ramp (smoothstep) based on how large the error is
        double t = Range.clip(absErr / kS_fullAtDeg, 0.0, 1.0);
        double smooth = (3.0 * t * t) - (2.0 * t * t * t);
        double kS_dyn = kS_min + (kS_max - kS_min) * smooth;

        // P + D + static assist
        double cmd = (kP * errorDeg) + (kD * dErrorDeg) + (kS_dyn * Math.signum(errorDeg));

        // Enforce minimum movement power outside deadband
        if (Math.abs(cmd) < MIN_MOVE_POWER) {
            cmd = MIN_MOVE_POWER * Math.signum(errorDeg);
        }

        return Range.clip(cmd, -MAX_POWER, MAX_POWER);
    }

    /**
     * Range to AprilTag in METERS using "proportions" instead of pose.
     * - Corner(VFOV) mode: uses pixel height from tag corners + VFOV model (more physics-based).
     * - Area(ta) mode: uses target area and a calibration constant (more empirical).
     */
    private Double getRangeToTagByProportions(LLResult result) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        LLResultTypes.FiducialResult chosen = null;

        if (targetId >= 0) {
            for (LLResultTypes.FiducialResult f : fiducials) {
                if (f.getFiducialId() == targetId) {
                    chosen = f;
                    break;
                }
            }
        } else {
            // auto-pick: largest target area (usually best/closest)
            double bestArea = -1;
            for (LLResultTypes.FiducialResult f : fiducials) {
                double area = f.getTargetArea();
                if (area > bestArea) {
                    bestArea = area;
                    chosen = f;
                }
            }
        }

        if (chosen == null) return null;

        return estimateRangeMetersFromArea(chosen);

    }

    /**
     * Option A: Estimate distance using tag pixel height from corners + vertical FOV.
     * distance ≈ (tagSize * fy) / pixelHeight
     */


    /**
     * Option B: Empirical distance using target area (ta).
     * distance ≈ AREA_K / sqrt(ta)
     */
    private Double estimateRangeMetersFromArea(LLResultTypes.FiducialResult tag) {
        double ta = tag.getTargetArea();
        if (ta <= 1e-6) return null;
        return AREA_K / Math.sqrt(ta);
    }
}