package org.firstinspires.ftc.teamcode.A_Regional;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * turret (refined)
 * - Reads Limelight tx/ty
 * - Drives CRServo so tx -> 0 (deadbanded)
 * - Smooth + consistent: tx low-pass, D-on-measurement, anti-windup, target flicker hold
 */
@Config
public class D_Turret {

    // ---------------- Dashboard Config ----------------
    // Limelight
    public static int POLL_RATE_HZ = 100;
    public static int PIPELINE_INDEX = 0;

    // Distance geometry
    public static double MOUNT_ANGLE_DEG = 11.0;
    public static double CAM_HEIGHT_IN   = 12.0;
    public static double POI_HEIGHT_IN   = 32.0;

    // Turret aiming behavior
    public static double DEADBAND_DEG = 1.0;
    public static double SERVO_SIGN = +1.0;

    // PID
    public static double KP = 0.014;
    public static double KI = 0.001;
    public static double KD = 0.003;

    // Integral management
    public static double I_SUM_CLAMP = 200.0;          // anti-windup clamp
    public static double I_ACTIVE_ERR_DEG = 8.0;       // only integrate when |err| <= this
    public static double I_DECAY_PER_SEC = 3.0;        // bleed iSum when far from target

    // Dynamic kS
    public static double KS_MIN = 0.02;
    public static double KS_MAX = 0.05;
    public static double KS_ERROR_SCALE = 0.01;

    // Output limits
    public static double MIN_POWER = 0.07;
    public static double MAX_POWER = 0.40;

    // Slew rate limiting (power per second)
    public static double SLEW_RATE_POWER_PER_SEC = 2.8;

    // tx filtering (0..1): higher = smoother but more lag (start 0.25–0.4)
    public static double TX_LP_ALPHA = 0.30;

    // target flicker handling
    public static double TARGET_HOLD_SEC = 0.02;       // keep last command briefly if LL flickers
    public static double HOLD_DECAY_PER_SEC = 4.0;     // decay held output toward 0 while holding
    // --------------------------------------------------

    private final Limelight3A ll;
    private CRServo turretServo;

    private boolean trackingEnabled = false;

    // Latest Limelight readings
    private boolean hasValidTarget = false;
    private double txDeg = 0.0;
    private double tyDeg = 0.0;

    // Filtered tx
    private boolean txFilterInit = false;
    private double txFiltDeg = 0.0;

    // Timing
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double tSec = 0.0;
    private double lastSeenSec = -999.0;

    // PID state
    private double iSum = 0.0;
    private double lastTxFiltDeg = 0.0; // for D-on-measurement
    private double lastOut = 0.0;

    /** Per request: only take llDevice in constructor. */
    public D_Turret(Limelight3A llDevice) {
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
        resetController();
        resetFilter();
    }

    /** TeleOp controls this. */
    public void setTrackingEnabled(boolean enabled) {
        if (enabled != trackingEnabled) {
            trackingEnabled = enabled;

            resetController();
            resetFilter();
            lastOut = 0.0;

            if (!trackingEnabled) stop();
        }
    }

    public boolean isTrackingEnabled() { return trackingEnabled; }

    /** Update tx/ty/valid from Limelight. Call every loop. */
    public void updateLimelight() {
        LLResult result = ll.getLatestResult();
        if (result != null && result.isValid()) {
            hasValidTarget = true;
            txDeg = result.getTx();
            tyDeg = result.getTy();
            lastSeenSec = tSec;
        } else {
            hasValidTarget = false;
        }
    }

    public boolean hasTarget() { return hasValidTarget; }
    public double getTxDeg() { return txDeg; }
    public double getTyDeg() { return tyDeg; }

    /** Filtered tx for telemetry/tuning. */
    public double getTxFilteredDeg() { return txFiltDeg; }

    /** Latest servo output command for telemetry/tuning. */
    public double getLastOut() { return lastOut; }

    /**
     * Horizontal (ground-plane) distance in inches:
     * distance = (POI_HEIGHT - CAM_HEIGHT) / tan(MOUNT_ANGLE + ty)
     */
    public double getGroundDistanceInches() {
        if (!hasValidTarget) return Double.NaN;

        double deltaH = POI_HEIGHT_IN - CAM_HEIGHT_IN;
        double angleDeg = MOUNT_ANGLE_DEG + tyDeg;
        double angleRad = Math.toRadians(angleDeg);

        double tan = Math.tan(angleRad);
        if (Math.abs(tan) < 1e-6) return Double.NaN;

        return deltaH / tan;
    }

    /**
     * Main turret control loop:
     * - call updateLimelight() before this each cycle
     * Returns true if aimed (within deadband) AND target valid/recent AND tracking enabled.
     */
    public boolean loop() {
        if (turretServo == null) return false;

        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0) dt = 1e-3;

        tSec += dt;

        if (!trackingEnabled) {
            stop();
            resetController();
            return false;
        }

        // If we have a valid target, update tx filter
        if (hasValidTarget) {
            if (!txFilterInit) {
                txFiltDeg = txDeg;
                lastTxFiltDeg = txFiltDeg;
                txFilterInit = true;
            } else {
                txFiltDeg = txFiltDeg + TX_LP_ALPHA * (txDeg - txFiltDeg);
            }
        }

        boolean recentlySeen = (tSec - lastSeenSec) <= TARGET_HOLD_SEC;

        // If no target (or not recently seen), hold output briefly (with decay), then stop
        if (!recentlySeen || !txFilterInit) {
            // decay last output toward zero while holding (prevents jerky stop/start)
            double decay = HOLD_DECAY_PER_SEC * dt;
            if (lastOut > 0) lastOut = Math.max(0, lastOut - decay);
            else             lastOut = Math.min(0, lastOut + decay);

            turretServo.setPower(lastOut);

            // If we truly haven't seen target, slowly bleed integral so it doesn't come back “biased”
            iSum = bleedTowardZero(iSum, I_DECAY_PER_SEC * dt);

            return false;
        }

        // Error uses filtered tx
        double error = 0.0 - txFiltDeg;

        // Deadband
        if (Math.abs(error) <= DEADBAND_DEG) {
            stop();
            resetController(); // keep filter; controller resets so it doesn't “kick” leaving deadband
            return true;
        }

        // Integral: only active near center; otherwise decay it
        if (Math.abs(error) <= I_ACTIVE_ERR_DEG) {
            iSum += error * dt;
            iSum = Range.clip(iSum, -I_SUM_CLAMP, I_SUM_CLAMP);
        } else {
            iSum = bleedTowardZero(iSum, I_DECAY_PER_SEC * dt);
        }

        // Derivative on measurement (less noise): d(error)/dt = -d(tx)/dt
        double txRate = (txFiltDeg - lastTxFiltDeg) / dt;
        lastTxFiltDeg = txFiltDeg;

        double p = KP * error;
        double i = KI * iSum;
        double d = KD * (-txRate);

        double pid = p + i + d;

        // Dynamic kS proportional to |error|
        double kS = KS_MIN + (KS_ERROR_SCALE * Math.abs(error));
        kS = Range.clip(kS, KS_MIN, KS_MAX);
        double ff = Math.signum(error) * kS;

        double rawOut = SERVO_SIGN * (pid + ff);

        // Clamp max
        rawOut = Range.clip(rawOut, -MAX_POWER, MAX_POWER);

        // Enforce minimum magnitude when moving
        if (Math.abs(rawOut) > 1e-6 && Math.abs(rawOut) < MIN_POWER) {
            rawOut = Math.signum(rawOut) * MIN_POWER;
        }

        // Slew limit
        double maxDelta = Math.abs(SLEW_RATE_POWER_PER_SEC) * dt;
        double limitedOut = slewLimit(lastOut, rawOut, maxDelta);
        lastOut = limitedOut;

        turretServo.setPower(limitedOut);
        return false;
    }

    /** Stops the turret. */
    public void stop() {
        if (turretServo != null) turretServo.setPower(0.0);
        lastOut = 0.0;
    }

    private void resetController() {
        iSum = 0.0;
    }

    private void resetFilter() {
        txFilterInit = false;
        txFiltDeg = 0.0;
        lastTxFiltDeg = 0.0;
        lastSeenSec = -999.0;
    }

    private static double slewLimit(double current, double target, double maxDelta) {
        double delta = target - current;
        if (delta > maxDelta) return current + maxDelta;
        if (delta < -maxDelta) return current - maxDelta;
        return target;
    }

    private static double bleedTowardZero(double value, double amount) {
        if (value > 0) return Math.max(0, value - amount);
        if (value < 0) return Math.min(0, value + amount);
        return 0;
    }
}