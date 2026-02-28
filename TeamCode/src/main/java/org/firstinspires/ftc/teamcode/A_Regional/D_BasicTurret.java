package org.firstinspires.ftc.teamcode.A_Regional;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * D_BasicTurret (PROVEN CONCEPT IMPLEMENTATION)
 *
 * Keeps the same public API you’ve been using:
 * - constructor(Limelight3A)
 * - init(CRServo), start()
 * - setTrackingEnabled(boolean), isTrackingEnabled()
 * - updateLimelight()
 * - loop() -> returns true when aimed (within deadband)
 * - stop()
 * - hasTarget(), getTxDeg(), getTyDeg(), getGroundDistanceInches()
 *
 * Proven approach for FTC CRServo + vision:
 *  - PD control on filtered tx (no I by default)
 *  - Static friction feedforward (kS) for consistent motion
 *  - Derivative-on-measurement using filtered tx rate (less noise)
 *  - Slew-rate limiting on output (smooth)
 *  - Short target-hold on flicker (prevents stop/start twitch)
 */
@Config
public class D_BasicTurret {

    // ---------------- Limelight ----------------
    public static int POLL_RATE_HZ = 100;
    public static int PIPELINE_INDEX = 0;

    // ---------------- Distance geometry ----------------
    public static double MOUNT_ANGLE_DEG = 11.0;
    public static double CAM_HEIGHT_IN   = 12.0;
    public static double POI_HEIGHT_IN   = 32.0;

    // ---------------- Behavior ----------------
    public static double SERVO_SIGN = +1.0;   // flip to -1 if turret spins wrong way
    public static double DEADBAND_DEG = 2.0;  // aimed if |tx| <= deadband

    // ---------------- Controller (PD + kS) ----------------
    // Start values: KP 0.02–0.06, KD 0.001–0.004 depending on noise and speed
    public static double KP = 0.013;
    public static double KD = 0.003;

    // Static friction feedforward: increase until it moves reliably at small tx (2–4 deg)
    public static double KS = 0.14;

    // Optional extra near-center friction boost to “finish” into deadband
    public static double KS_NEAR = 0.16;
    public static double KS_NEAR_RANGE_DEG = 3.0;

    // ---------------- Filtering ----------------
    // tx low-pass smoothing (0..1): higher = follows raw more, lower = smoother
    public static double TX_LP_ALPHA = .6;     // 0.2–0.45 typical
    // tx rate low-pass smoothing for stable derivative
    public static double RATE_LP_ALPHA = .5;   // 0.2–0.5 typical

    // ---------------- Output limits ----------------
    public static double MAX_POWER = 0.45;
    public static double MIN_POWER = 0.2;       // helps overcome stiction
    public static double SLEW_POWER_PER_SEC = 100;

    // ---------------- Target handling ----------------
    public static double TARGET_HOLD_SEC = 0.02;
    public static double LOST_OUTPUT_DECAY_PER_SEC = 1;

    // ---------------- Timing clamps ----------------
    public static double DT_MIN = 1e-3;
    public static double DT_MAX = 0.08;
    // --------------------------------------------------

    private final Limelight3A ll;
    private CRServo turretServo;

    private boolean trackingEnabled = false;

    // Latest Limelight readings
    private boolean hasValidTarget = false;
    private double txDeg = 0.0;
    private double tyDeg = 0.0;

    // Time
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double tSec = 0.0;
    private double lastSeenSec = -999.0;

    // Filtered signals
    private boolean filtInit = false;
    private double txFilt = 0.0;
    private double txRateFilt = 0.0;
    private double lastTxFilt = 0.0;

    // Output state (slew/hold)
    private double out = 0.0;

    /** Only take Limelight3A in constructor (same as your old class style). */
    public D_BasicTurret(Limelight3A llDevice) {
        this.ll = llDevice;
        loopTimer.reset();
    }

    /** Bind the servo once (call in init). */
    public void init(CRServo servo) {
        this.turretServo = servo;
    }

    /** Call once in init() after init(servo). */
    public void start() {
        ll.setPollRateHz(POLL_RATE_HZ);
        ll.pipelineSwitch(PIPELINE_INDEX);
        ll.start();

        loopTimer.reset();
        tSec = 0.0;
        resetFilters();
        out = 0.0;
        lastSeenSec = -999.0;
    }

    /** TeleOp controls this. */
    public void setTrackingEnabled(boolean enabled) {
        if (enabled != trackingEnabled) {
            trackingEnabled = enabled;
            resetFilters();
            out = 0.0;
            if (!trackingEnabled) stop();
        }
    }

    public boolean isTrackingEnabled() { return trackingEnabled; }

    /** Update tx/ty/valid from Limelight. Call every loop. */
    public void updateLimelight() {
        LLResult r = ll.getLatestResult();
        if (r != null && r.isValid()) {
            hasValidTarget = true;
            txDeg = r.getTx();
            tyDeg = r.getTy();
            lastSeenSec = tSec;
        } else {
            hasValidTarget = false;
        }
    }

    public boolean hasTarget() { return hasValidTarget; }
    public double getTxDeg() { return txDeg; }
    public double getTyDeg() { return tyDeg; }

    /**
     * Horizontal (ground-plane) distance in inches:
     * distance = (POI_HEIGHT - CAM_HEIGHT) / tan(MOUNT_ANGLE + ty)
     */
    public double getGroundDistanceInches() {
        if (!hasValidTarget) return Double.NaN;

        double deltaH = POI_HEIGHT_IN - CAM_HEIGHT_IN;
        double angleDeg = MOUNT_ANGLE_DEG + tyDeg;

        double tan = Math.tan(Math.toRadians(angleDeg));
        if (Math.abs(tan) < 1e-6) return Double.NaN;

        return deltaH / tan;
    }

    /**
     * Main turret loop:
     * - call updateLimelight() before this each cycle
     * - returns true when aimed (within deadband) AND tracking enabled AND target seen recently
     */
    public boolean loop() {
        if (turretServo == null) return false;

        // dt
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt < DT_MIN) dt = DT_MIN;
        if (dt > DT_MAX) dt = DT_MAX;

        tSec += dt;

        if (!trackingEnabled) {
            stop();
            return false;
        }

        boolean recentlySeen = (tSec - lastSeenSec) <= TARGET_HOLD_SEC;

        // Update filters only when we have a fresh target
        if (hasValidTarget) {
            if (!filtInit) {
                txFilt = txDeg;
                lastTxFilt = txFilt;
                txRateFilt = 0.0;
                filtInit = true;
            } else {
                // Low-pass tx
                txFilt = txFilt + TX_LP_ALPHA * (txDeg - txFilt);

                // Derivative of filtered tx (deg/s)
                double txRate = (txFilt - lastTxFilt) / dt;
                lastTxFilt = txFilt;

                // Low-pass rate
                txRateFilt = txRateFilt + RATE_LP_ALPHA * (txRate - txRateFilt);
            }
        }

        // If target flickers, decay output smoothly to zero
        if (!recentlySeen || !filtInit) {
            out = decayTowardZero(out, LOST_OUTPUT_DECAY_PER_SEC * dt);
            turretServo.setPower(out);
            return false;
        }

        // Error uses filtered tx
        double error = -txFilt;
        double absErr = Math.abs(error);

        // Deadband: stop output but don't reset filters
        if (absErr <= DEADBAND_DEG) {
            out = 0.0;
            turretServo.setPower(0.0);
            return true;
        }

        // PD (derivative-on-measurement): d(error)/dt = -d(tx)/dt
        double p = KP * error;
        double d = KD * (-txRateFilt);

        // Static friction FF (kS) with optional near-center boost
        double ksNearBlend = 1.0 - Range.clip(absErr / Math.max(1e-6, KS_NEAR_RANGE_DEG), 0.0, 1.0);
        double ksTotal = KS + (KS_NEAR * ksNearBlend);
        double ff = Math.signum(error) * ksTotal;

        double raw = SERVO_SIGN * (p + d + ff);

        // Clamp max
        raw = Range.clip(raw, -MAX_POWER, MAX_POWER);

        // Minimum power when moving
        if (Math.abs(raw) > 1e-6 && Math.abs(raw) < MIN_POWER) {
            raw = Math.signum(raw) * MIN_POWER;
        }

        // Slew limit
        double maxDelta = Math.abs(SLEW_POWER_PER_SEC) * dt;
        out = slewLimit(out, raw, maxDelta);

        turretServo.setPower(out);
        return false;
    }

    /** Stops the turret immediately. */
    public void stop() {
        if (turretServo != null) turretServo.setPower(0.0);
        out = 0.0;
    }

    // ---------------- Helpers ----------------
    private void resetFilters() {
        filtInit = false;
        txFilt = 0.0;
        txRateFilt = 0.0;
        lastTxFilt = 0.0;
    }

    private static double slewLimit(double current, double target, double maxDelta) {
        double delta = target - current;
        if (delta > maxDelta) return current + maxDelta;
        if (delta < -maxDelta) return current - maxDelta;
        return target;
    }

    private static double decayTowardZero(double value, double amount) {
        if (value > 0) return Math.max(0.0, value - amount);
        if (value < 0) return Math.min(0.0, value + amount);
        return 0.0;
    }
}