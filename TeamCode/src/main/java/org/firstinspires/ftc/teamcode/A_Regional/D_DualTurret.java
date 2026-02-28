package org.firstinspires.ftc.teamcode.A_Regional;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * D_DualTurret (degree-first)
 *
 * Two-stage turret tracking:
 *  - Odometry stage: uses drivetrain heading + turret encoder to aim toward a FIELD goal heading (all degrees)
 *  - Vision stage: once odom close (gate) AND Limelight sees target, uses tx (degrees) to finish
 *
 * Conventions:
 *  - All tuning constants / telemetry / error signals are in DEGREES.
 *  - Only convert to radians for trig or wrap helper if you want, but we implement wrap in degrees directly.
 *
 * Hardware:
 *  - CRServo turret
 *  - REV Through Bore encoder read via DcMotorEx (configured as "turretEncoder")
 *
 * Gear ratio for ticks->turret output:
 *  - Encoder axle drives turret through 31T -> 112T
 *  - turretAngle = encoderAngle * (31/112) * ENCODER_SIGN
 *  - (44:28 is upstream of encoder, does NOT affect mapping)
 */
@Config
public class D_DualTurret {

    // ---------------- Modes ----------------
    public enum Mode {
        LOCK_TO_DRIVETRAIN,  // turret rel heading -> 0 deg
        ODOMETRY_ONLY,       // turret rel heading -> (goalFieldDeg - drivetrainHeadingDeg)
        LIMELIGHT_ONLY,      // tx only
        DUAL_STAGE           // odom until gate, then limelight when target seen/recent
    }

    // ---------------- Limelight ----------------
    public static int POLL_RATE_HZ = 100;
    public static int PIPELINE_INDEX = 0;

    // ---------------- Direction ----------------
    public static double SERVO_SIGN = +1.0;     // flip if turret spins wrong
    public static double ENCODER_SIGN = +1.0;   // flip if turret angle increases wrong way

    // ---------------- Encoder ----------------
    public static double ENCODER_TICKS_PER_REV = 8192.0;   // confirm your actual CPR
    public static double ENCODER_ZERO_TICKS = 0.0;

    // Encoder axle -> turret output ratio (31:112)
    public static double ENCODER_TO_TURRET_RATIO = 31.0 / 112.0;

    // ---------------- Goal heading (FIELD frame) ----------------
    public static double GOAL_FIELD_HEADING_DEG = 0.0;

    // ---------------- Odom controller (degrees) ----------------
    public static double ODOM_DEADBAND_DEG = 2.0;
    public static double ODOM_MAX_POWER = 0.55;
    public static double ODOM_MIN_POWER = 0.18;
    public static double ODOM_SLEW_POWER_PER_SEC = 100;

    // PD + kS (error is degrees, rate is deg/s)
    public static double ODOM_KP = 0.020;   // power/deg  (start 0.010–0.040)
    public static double ODOM_KD = 0.0015;  // power/(deg/s) (start 0.0008–0.003)
    public static double ODOM_KS = 0.10;
    public static double ODOM_KS_NEAR = 0.13;
    public static double ODOM_KS_NEAR_RANGE_DEG = 5.0;

    // Filtering (degrees)
    public static double ODOM_ANGLE_LP_ALPHA = 0.6;
    public static double ODOM_RATE_LP_ALPHA  = 0.5;

    // ---------------- Vision controller (degrees) ----------------
    public static double V_DEADBAND_DEG = 2.0;
    public static double V_MAX_POWER = 0.45;
    public static double V_MIN_POWER = 0.20;
    public static double V_SLEW_POWER_PER_SEC = 100;

    // PD + kS (tx degrees, txRate deg/s)
    public static double V_KP = 0.013;
    public static double V_KD = 0.003;
    public static double V_KS = 0.14;
    public static double V_KS_NEAR = 0.16;
    public static double V_KS_NEAR_RANGE_DEG = 3.0;

    public static double V_TX_LP_ALPHA   = 0.6;
    public static double V_RATE_LP_ALPHA = 0.5;

    // Target hold & decay
    public static double TARGET_HOLD_SEC = 0.02;
    public static double LOST_OUTPUT_DECAY_PER_SEC = 1.0;

    // Dual-stage gate (degrees)
    public static double ODOM_TO_VISION_GATE_DEG = 8.0;

    // Timing clamps
    public static double DT_MIN = 1e-3;
    public static double DT_MAX = 0.08;

    // ---------------- Internal state ----------------
    private final Limelight3A ll;
    private CRServo turretServo;
    private DcMotorEx encoderMotor;

    private boolean trackingEnabled = false;
    private Mode mode = Mode.DUAL_STAGE;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double tSec = 0.0;

    // drivetrain heading (deg, field frame)
    private double drivetrainHeadingDeg = 0.0;

    // limelight
    private boolean hasValidTarget = false;
    private double txDeg = 0.0;
    private double tyDeg = 0.0;
    private double lastSeenSec = -999.0;

    // odom filter state (we filter turretRelDeg and derive rate)
    private boolean odomFiltInit = false;
    private double turretRelDegFilt = 0.0;
    private double turretRelRateDegPerSecFilt = 0.0;
    private double lastTurretRelDegFilt = 0.0;

    // vision filter state
    private boolean vFiltInit = false;
    private double txFilt = 0.0;
    private double txRateDegPerSecFilt = 0.0;
    private double lastTxFilt = 0.0;

    // output (slew-limited)
    private double out = 0.0;

    public D_DualTurret(Limelight3A llDevice) {
        this.ll = llDevice;
        loopTimer.reset();
    }

    public void init(CRServo servo, DcMotorEx encoderMotor) {
        this.turretServo = servo;
        this.encoderMotor = encoderMotor;
    }

    public void start() {
        ll.setPollRateHz(POLL_RATE_HZ);
        ll.pipelineSwitch(PIPELINE_INDEX);
        ll.start();

        loopTimer.reset();
        tSec = 0.0;
        out = 0.0;
        lastSeenSec = -999.0;
        resetOdomFilters();
        resetVisionFilters();
    }

    public void stop() {
        if (turretServo != null) turretServo.setPower(0.0);
        out = 0.0;
    }

    public void setTrackingEnabled(boolean enabled) {
        if (enabled != trackingEnabled) {
            trackingEnabled = enabled;
            out = 0.0;
            resetOdomFilters();
            resetVisionFilters();
            if (!trackingEnabled) stop();
        }
    }

    public boolean isTrackingEnabled() { return trackingEnabled; }

    public void setMode(Mode m) {
        if (m != mode) {
            mode = m;
            out = 0.0;
            resetOdomFilters();
            resetVisionFilters();
        }
    }

    public Mode getMode() { return mode; }

    public void setGoalFieldHeadingDeg(double deg) { GOAL_FIELD_HEADING_DEG = deg; }

    public boolean hasTarget() { return hasValidTarget; }
    public double getTxDeg() { return txDeg; }
    public double getTyDeg() { return tyDeg; }

    // ---- Useful for telemetry ----
    public double getDrivetrainHeadingDeg() { return drivetrainHeadingDeg; }

    /**
     * Call once per loop.
     * drivetrainHeadingDeg: RoadRunner pose heading converted to degrees in TeleOp
     *
     * Returns true when aimed under current active controller.
     */
    public boolean update(double drivetrainHeadingDeg) {
        if (turretServo == null || encoderMotor == null) return false;

        // dt
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt < DT_MIN) dt = DT_MIN;
        if (dt > DT_MAX) dt = DT_MAX;
        tSec += dt;

        this.drivetrainHeadingDeg = wrapDeg(drivetrainHeadingDeg);

        if (!trackingEnabled) {
            stop();
            return false;
        }

        updateLimelight();

        // Determine odometry target relative heading (degrees)
        double targetRelDeg;
        switch (mode) {
            case LOCK_TO_DRIVETRAIN:
                targetRelDeg = 0.0;
                break;
            case ODOMETRY_ONLY:
            case DUAL_STAGE:
                targetRelDeg = wrapDeg(GOAL_FIELD_HEADING_DEG - this.drivetrainHeadingDeg);
                break;
            case LIMELIGHT_ONLY:
            default:
                targetRelDeg = 0.0; // unused
                break;
        }

        // Current turret relative heading from encoder (degrees)
        double turretRelDeg = getTurretRelativeHeadingDeg();

        // Update odom filters
        updateOdomFilters(turretRelDeg, dt);

        // Decide controller
        boolean useVision = (mode == Mode.LIMELIGHT_ONLY);

        if (mode == Mode.DUAL_STAGE) {
            double odomErrDeg = wrapDeg(targetRelDeg - turretRelDeg);
            boolean gateReached = Math.abs(odomErrDeg) <= ODOM_TO_VISION_GATE_DEG;

            boolean recentlySeen = (tSec - lastSeenSec) <= TARGET_HOLD_SEC;
            boolean visionAvailable = hasValidTarget || recentlySeen;

            useVision = gateReached && visionAvailable;
        }

        if (mode == Mode.LIMELIGHT_ONLY) {
            return stepVision(dt);
        }

        if (useVision) {
            return stepVision(dt);
        } else {
            return stepOdom(targetRelDeg, turretRelDeg, dt);
        }
    }

    // ---------------- Limelight ----------------
    private void updateLimelight() {
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

    // ---------------- Odom control (degrees) ----------------
    private boolean stepOdom(double targetRelDeg, double turretRelDeg, double dt) {
        double errDeg = wrapDeg(targetRelDeg - turretRelDeg);
        double absErr = Math.abs(errDeg);

        if (absErr <= ODOM_DEADBAND_DEG) {
            out = 0.0;
            turretServo.setPower(0.0);
            return true;
        }

        // derivative-on-measurement:
        // error = target - turret; d(error)/dt = - d(turret)/dt
        double turretRateDegPerSec = turretRelRateDegPerSecFilt;

        double p = ODOM_KP * errDeg;
        double d = ODOM_KD * (-turretRateDegPerSec);

        // kS with near blend
        double ksNearBlend = 1.0 - Range.clip(absErr / Math.max(1e-6, ODOM_KS_NEAR_RANGE_DEG), 0.0, 1.0);
        double ksTotal = ODOM_KS + (ODOM_KS_NEAR * ksNearBlend);
        double ff = Math.signum(errDeg) * ksTotal;

        double raw = SERVO_SIGN * (p + d + ff);

        raw = Range.clip(raw, -ODOM_MAX_POWER, ODOM_MAX_POWER);

        if (Math.abs(raw) > 1e-6 && Math.abs(raw) < ODOM_MIN_POWER) {
            raw = Math.signum(raw) * ODOM_MIN_POWER;
        }

        double maxDelta = Math.abs(ODOM_SLEW_POWER_PER_SEC) * dt;
        out = slewLimit(out, raw, maxDelta);

        turretServo.setPower(out);
        return false;
    }

    private void updateOdomFilters(double turretRelDeg, double dt) {
        if (!odomFiltInit) {
            turretRelDegFilt = turretRelDeg;
            lastTurretRelDegFilt = turretRelDegFilt;
            turretRelRateDegPerSecFilt = 0.0;
            odomFiltInit = true;
            return;
        }

        // Filter angle with shortest wrap delta
        double delta = wrapDeg(turretRelDeg - turretRelDegFilt);
        turretRelDegFilt = wrapDeg(turretRelDegFilt + ODOM_ANGLE_LP_ALPHA * delta);

        double rate = wrapDeg(turretRelDegFilt - lastTurretRelDegFilt) / dt;
        lastTurretRelDegFilt = turretRelDegFilt;

        turretRelRateDegPerSecFilt =
                turretRelRateDegPerSecFilt + ODOM_RATE_LP_ALPHA * (rate - turretRelRateDegPerSecFilt);
    }

    // ---------------- Vision control (degrees) ----------------
    private boolean stepVision(double dt) {
        boolean recentlySeen = (tSec - lastSeenSec) <= TARGET_HOLD_SEC;

        if (hasValidTarget) {
            if (!vFiltInit) {
                txFilt = txDeg;
                lastTxFilt = txFilt;
                txRateDegPerSecFilt = 0.0;
                vFiltInit = true;
            } else {
                txFilt = txFilt + V_TX_LP_ALPHA * (txDeg - txFilt);

                double rate = (txFilt - lastTxFilt) / dt;
                lastTxFilt = txFilt;

                txRateDegPerSecFilt =
                        txRateDegPerSecFilt + V_RATE_LP_ALPHA * (rate - txRateDegPerSecFilt);
            }
        }

        if (!recentlySeen || !vFiltInit) {
            out = decayTowardZero(out, LOST_OUTPUT_DECAY_PER_SEC * dt);
            turretServo.setPower(out);
            return false;
        }

        double error = -txFilt;
        double absErr = Math.abs(error);

        if (absErr <= V_DEADBAND_DEG) {
            out = 0.0;
            turretServo.setPower(0.0);
            return true;
        }

        double p = V_KP * error;
        double d = V_KD * (-txRateDegPerSecFilt);

        double ksNearBlend = 1.0 - Range.clip(absErr / Math.max(1e-6, V_KS_NEAR_RANGE_DEG), 0.0, 1.0);
        double ksTotal = V_KS + (V_KS_NEAR * ksNearBlend);
        double ff = Math.signum(error) * ksTotal;

        double raw = SERVO_SIGN * (p + d + ff);

        raw = Range.clip(raw, -V_MAX_POWER, V_MAX_POWER);

        if (Math.abs(raw) > 1e-6 && Math.abs(raw) < V_MIN_POWER) {
            raw = Math.signum(raw) * V_MIN_POWER;
        }

        double maxDelta = Math.abs(V_SLEW_POWER_PER_SEC) * dt;
        out = slewLimit(out, raw, maxDelta);

        turretServo.setPower(out);
        return false;
    }

    // ---------------- Encoder math (degrees) ----------------
    /** Turret relative heading in degrees (-180, +180]. */
    public double getTurretRelativeHeadingDeg() {
        double ticks = encoderMotor.getCurrentPosition() - ENCODER_ZERO_TICKS;

        double encoderRev = ticks / ENCODER_TICKS_PER_REV;
        double encoderDeg = encoderRev * 360.0;

        // turretDeg = encoderDeg * ratio * sign
        double turretDeg = encoderDeg * ENCODER_TO_TURRET_RATIO * ENCODER_SIGN;

        return wrapDeg(turretDeg);
    }

    /** Convert desired turret relative deg to encoder ticks target (for debugging / future position hold use). */
    public double turretRelDegToEncoderTicks(double turretRelDeg) {
        double encoderDeg = turretRelDeg / (ENCODER_TO_TURRET_RATIO * ENCODER_SIGN);
        double encoderRev = encoderDeg / 360.0;
        return ENCODER_ZERO_TICKS + encoderRev * ENCODER_TICKS_PER_REV;
    }

    // ---------------- Helpers ----------------
    private void resetOdomFilters() {
        odomFiltInit = false;
        turretRelDegFilt = 0.0;
        lastTurretRelDegFilt = 0.0;
        turretRelRateDegPerSecFilt = 0.0;
    }

    private void resetVisionFilters() {
        vFiltInit = false;
        txFilt = 0.0;
        txRateDegPerSecFilt = 0.0;
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

    /** Wrap degrees to (-180, +180]. */
    public static double wrapDeg(double deg) {
        while (deg <= -180.0) deg += 360.0;
        while (deg >  180.0) deg -= 360.0;
        return deg;
    }
}