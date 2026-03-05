package org.firstinspires.ftc.teamcode.A_Regional;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * D_DualTurret (Dual-stage ODOM -> LL) using the SAME output shaping as your proven D_BasicTurret:
 *  - low-pass filtered error
 *  - derivative-on-measurement (filtered rate)
 *  - kS + optional near-center boost
 *  - slew-rate limiting
 *  - short target-hold / decay-on-loss
 *
 * This version uses:
 *  - Drivetrain yaw (passed in each loop)
 *  - Turret IMU yaw ("turretImu") for turret ABS heading
 *  - POI (goalX, goalY) for ODOM target ABS heading
 *  - Limelight tx for fine aiming once armed/valid
 *
 * Public API intentionally similar to D_BasicTurret:
 *  - constructor(Limelight3A, IMU)
 *  - init(CRServo), start()
 *  - setTrackingEnabled(boolean), isTrackingEnabled()
 *  - updateLimelight()
 *  - loop(pose, drivetrainYawRad)  // active dual-stage
 *  - stop()
 *
 * Extra:
 *  - syncTurretImuToDrivetrain(drivetrainYawRad): sets an offset so turret ABS matches drivetrain yaw NOW.
 *  - resetTurretImuYawAndSync(drivetrainYawRad): resetYaw() then syncs offset.
 */
@Config
public class D_DualTurret {

    // ---------------- POI / ODOM target ----------------
    public static double goalX = 51.0;
    public static double goalY = 41.0;

    // ---------------- Limelight ----------------
    public static int POLL_RATE_HZ = 100;
    public static int PIPELINE_INDEX = 0;

    // ---------------- Behavior ----------------
    public static double SERVO_SIGN = +1.0;
    public static double DEADBAND_DEG = 2.0; // used for BOTH odom and LL "aimed" definition

    // ---------------- Dual-stage switching ----------------
    public static boolean ENABLE_DUAL_STAGE = true;
    public static double ARM_LL_DEG = 1.4;        // when |odom error| <= this, start LL + allow LL stage
    public static double LL_LOST_TIMEOUT_SEC = 0.15; // drop back to ODOM if LL target lost this long
    public static double ODOM_ONLY_WHEN_NO_POSE_SEC = 0.0; // kept for safety; 0 means immediate fallback behavior

    // ---------------- Turret IMU ----------------
    // turretAbsRad = wrap( turretImuYawRad + turretImuOffsetRad )
    public static double turretImuOffsetRad = 0.0;
    public static double TURRET_IMU_SIGN = +1.0; // set -1 if turret IMU yaw is reversed

    // ---------------- Soft limiter (relative to drivetrain yaw) ----------------
    public static boolean ENABLE_SOFT_LIMIT = true;
    public static double MAX_REL_ANGLE_DEG = 90.0;
    public static double LIMIT_CUSHION_DEG = 1.0;

    // ---------------- Controller shaping (same style as D_BasicTurret) ----------------
    // ODOM gains
    public static double ODOM_KP = 0.020;
    public static double ODOM_KD = 0.004;
    public static double ODOM_KS = 0.14;
    public static double ODOM_KS_NEAR = 0.16;
    public static double ODOM_KS_NEAR_RANGE_DEG = 3.0;

    // LL gains
    public static double LL_KP = 0.013;
    public static double LL_KD = 0.003;
    public static double LL_KS = 0.14;
    public static double LL_KS_NEAR = 0.16;
    public static double LL_KS_NEAR_RANGE_DEG = 3.0;

    // Filtering (same semantics as your working class)
    public static double ERR_LP_ALPHA = 0.6;     // like TX_LP_ALPHA but for generic error signal
    public static double RATE_LP_ALPHA = 0.5;    // same as before

    // Output limits (same semantics as your working class)
    public static double MAX_POWER = 0.45;
    public static double MIN_POWER = 0.20;
    public static double SLEW_POWER_PER_SEC = 100;

    // Target handling / decay (same semantics as your working class)
    public static double TARGET_HOLD_SEC = 0.02;          // for LL flicker hold
    public static double LOST_OUTPUT_DECAY_PER_SEC = 1.0; // decay toward 0 on loss

    // Timing clamps
    public static double DT_MIN = 1e-3;
    public static double DT_MAX = 0.08;

    // ---------------- Modes ----------------
    public enum Mode {
        AUTO_DUAL,      // ODOM -> LL when close and target is valid
        ODOM_ONLY,      // never uses LL (still can start/stop LL if you want, but not used for control)
        LL_ONLY,        // uses tx only (like your D_BasicTurret)
        LOCK_TO_DRIVE,  // turret holds drivetrain heading (rel -> 0)
        OFF
    }

    // ---------------- Hardware ----------------
    private final Limelight3A ll;
    private final IMU turretImu;
    private CRServo turretServo;

    // ---------------- State ----------------
    private boolean trackingEnabled = false;
    private Mode mode = Mode.AUTO_DUAL;

    // Limelight readings
    private boolean hasValidTarget = false;
    private double txDeg = 0.0;
    private double tyDeg = 0.0;

    // time
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double tSec = 0.0;
    private double lastSeenSec = -999.0;
    private double lastLLOkSec = -999.0;

    // stage
    private enum Stage { ODOM, LL }
    private Stage stage = Stage.ODOM;

    // filtered signals (generic error filtering)
    private boolean filtInit = false;
    private double errFilt = 0.0;
    private double errRateFilt = 0.0;
    private double lastErrFilt = 0.0;

    // output state
    private double out = 0.0;

    // debug/telemetry
    private double lastErrDeg = 0.0;
    private double lastErrRateDegPerSec = 0.0;
    private double lastOdomErrDeg = 0.0;
    private double lastLLErrDeg = 0.0;

    public D_DualTurret(Limelight3A llDevice, IMU turretImu) {
        this.ll = llDevice;
        this.turretImu = turretImu;
        loopTimer.reset();
    }

    public void init(CRServo servo) {
        this.turretServo = servo;
    }

    public void start() {
        ll.setPollRateHz(POLL_RATE_HZ);
        ll.pipelineSwitch(PIPELINE_INDEX);
        ll.start();

        loopTimer.reset();
        tSec = 0.0;
        lastSeenSec = -999.0;
        lastLLOkSec = -999.0;
        stage = Stage.ODOM;

        resetFilters();
        out = 0.0;
    }

    public void stop() {
        if (turretServo != null) turretServo.setPower(0.0);
        out = 0.0;
    }

    public void setTrackingEnabled(boolean enabled) {
        if (enabled != trackingEnabled) {
            trackingEnabled = enabled;
            stage = Stage.ODOM;
            resetFilters();
            out = 0.0;
            if (!trackingEnabled) stop();
        }
    }

    public boolean isTrackingEnabled() { return trackingEnabled; }

    public void setMode(Mode m) {
        if (m != mode) {
            mode = m;
            stage = Stage.ODOM;
            resetFilters();
            out = 0.0;
        }
    }

    public Mode getMode() { return mode; }
    public String getStage() { return stage.name(); }

    // ----- Limelight update (same idea as your Basic) -----
    public void updateLimelight() {
        LLResult r = ll.getLatestResult();
        if (r != null && r.isValid()) {
            hasValidTarget = true;
            txDeg = r.getTx();
            tyDeg = r.getTy();
            lastSeenSec = tSec;
            lastLLOkSec = tSec;
        } else {
            hasValidTarget = false;
        }
    }

    public boolean hasTarget() { return hasValidTarget; }
    public double getTxDeg() { return txDeg; }
    public double getTyDeg() { return tyDeg; }

    // ----- IMU heading helpers -----
    /** Turret absolute yaw in radians (with sign + offset). */
    public double getTurretAbsYawRad() {
        YawPitchRollAngles a = turretImu.getRobotYawPitchRollAngles();
        double yaw = a.getYaw(AngleUnit.RADIANS);
        return wrapAngle(TURRET_IMU_SIGN * yaw + turretImuOffsetRad);
    }

    /**
     * Matches turret ABS heading to drivetrain heading RIGHT NOW (no resetYaw required).
     * After calling this, getTurretAbsYawRad() will equal drivetrainYawRad (approximately).
     */
    public void syncTurretImuToDrivetrain(double drivetrainYawRad) {
        double rawTurretYaw = turretImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        turretImuOffsetRad = wrapAngle(drivetrainYawRad - (TURRET_IMU_SIGN * rawTurretYaw));
    }

    /**
     * Hard reset turret IMU yaw (sets it to 0), then sets offset so turret ABS matches drivetrain yaw.
     * Use when you want a clean reference at init or when driver requests re-sync.
     */
    public void resetTurretImuYawAndSync(double drivetrainYawRad) {
        turretImu.resetYaw();
        // after reset, raw turret yaw is ~0
        turretImuOffsetRad = wrapAngle(drivetrainYawRad);
    }

    /** Convenience: relative turret angle to robot (deg) = wrap(turretAbs - drivetrainYaw). */
    public double getTurretRelDeg(double drivetrainYawRad) {
        double rel = wrapAngle(getTurretAbsYawRad() - drivetrainYawRad);
        return Math.toDegrees(rel);
    }

    // ----- Dual-stage loop -----

    /**
     * Dual-stage tracking loop.
     * Call updateLimelight() BEFORE loop() each cycle, like your Basic class.
     *
     * @param pose RR pose (x,y used for POI bearing)
     * @param drivetrainYawRad drivetrain IMU yaw (rad)
     * @return true when "aimed" (deadband) for the active stage
     */
    public boolean loop(Pose2d pose, double drivetrainYawRad) {
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

        // compute headings
        double turretAbsRad = getTurretAbsYawRad();
        double turretRelRad = wrapAngle(turretAbsRad - drivetrainYawRad);

        // soft limiter gating is done on OUTPUT (using rel angle)
        // compute ODOM target (bearing to POI)
        boolean havePose = (pose != null);
        double targetAbsRad = turretAbsRad;
        if (havePose) {
            targetAbsRad = Math.atan2(goalY - pose.position.y, goalX - pose.position.x);
        }

        // mode forces
        if (mode == Mode.OFF) {
            stop();
            return false;
        }
        if (mode == Mode.LOCK_TO_DRIVE) {
            stage = Stage.ODOM; // still uses odom-shaped controller
            double errDeg = Math.toDegrees(wrapAngle(drivetrainYawRad - turretAbsRad));
            return runController(errDeg, dt, turretRelRad, /*useLLGains*/false, /*recentlySeen*/true);
        }

        // Determine stage for AUTO_DUAL
        if (!ENABLE_DUAL_STAGE || mode == Mode.ODOM_ONLY) {
            stage = Stage.ODOM;
        } else if (mode == Mode.LL_ONLY) {
            stage = Stage.LL;
        } else { // AUTO_DUAL
            // Arm LL when odom error is small
            double odomErrDeg = Math.toDegrees(wrapAngle(targetAbsRad - turretAbsRad));
            if (stage == Stage.ODOM) {
                if (Math.abs(odomErrDeg) <= ARM_LL_DEG) {
                    // if LL valid now, switch to LL
                    if (hasValidTarget) {
                        stage = Stage.LL;
                        resetFilters();
                    }
                }
            } else { // stage == LL
                // fall back if LL lost too long
                double secSinceOk = tSec - lastLLOkSec;
                if (!hasValidTarget && secSinceOk > LL_LOST_TIMEOUT_SEC) {
                    stage = Stage.ODOM;
                    resetFilters();
                }
            }
        }

        // Recently seen logic for LL (matches your Basic)
        boolean recentlySeen = (tSec - lastSeenSec) <= TARGET_HOLD_SEC;

        // Compute error signal (deg) based on stage
        double errDeg;
        boolean useLLGains;

        if (stage == Stage.LL) {
            // use filtered tx like your basic (error = -tx)
            // BUT we still allow brief hold on flicker
            if (!recentlySeen) {
                // decay out when LL flickers out
                out = decayTowardZero(out, LOST_OUTPUT_DECAY_PER_SEC * dt);
                turretServo.setPower(out);
                lastErrDeg = 0.0;
                lastErrRateDegPerSec = 0.0;
                lastLLErrDeg = 0.0;
                return false;
            }
            errDeg = -txDeg;
            useLLGains = true;
            lastLLErrDeg = errDeg;
        } else {
            // ODOM stage: error is absolute heading error to POI
            errDeg = Math.toDegrees(wrapAngle(targetAbsRad - turretAbsRad));
            useLLGains = false;
            lastOdomErrDeg = errDeg;
            // ODOM does not depend on LL visibility
            recentlySeen = true;
        }

        // Run the shared shaped controller
        return runController(errDeg, dt, turretRelRad, useLLGains, recentlySeen);
    }

    /**
     * Shared shaped controller (same structure as D_BasicTurret):
     *  - filter error (deg)
     *  - filter error rate (deg/s)
     *  - PD + kS (+ near boost)
     *  - clamp + min power + slew limit
     *  - optional soft-limit gate (rel angle)
     */
    private boolean runController(double errDegRaw,
                                  double dt,
                                  double turretRelRad,
                                  boolean useLLGains,
                                  boolean allowControl) {

        if (!allowControl) {
            out = decayTowardZero(out, LOST_OUTPUT_DECAY_PER_SEC * dt);
            turretServo.setPower(out);
            return false;
        }

        // init / update filters
        if (!filtInit) {
            errFilt = errDegRaw;
            lastErrFilt = errFilt;
            errRateFilt = 0.0;
            filtInit = true;
        } else {
            // low-pass error
            errFilt = errFilt + ERR_LP_ALPHA * (errDegRaw - errFilt);

            // derivative of filtered error (deg/s)
            double rate = (errFilt - lastErrFilt) / dt;
            lastErrFilt = errFilt;

            // low-pass the rate
            errRateFilt = errRateFilt + RATE_LP_ALPHA * (rate - errRateFilt);
        }

        double error = errFilt;
        double absErr = Math.abs(error);

        // deadband
        if (absErr <= DEADBAND_DEG) {
            out = 0.0;
            turretServo.setPower(0.0);
            lastErrDeg = error;
            lastErrRateDegPerSec = errRateFilt;
            return true;
        }

        // select gains
        double kp = useLLGains ? LL_KP : ODOM_KP;
        double kd = useLLGains ? LL_KD : ODOM_KD;

        double ks = useLLGains ? LL_KS : ODOM_KS;
        double ksNear = useLLGains ? LL_KS_NEAR : ODOM_KS_NEAR;
        double ksNearRange = useLLGains ? LL_KS_NEAR_RANGE_DEG : ODOM_KS_NEAR_RANGE_DEG;

        // PD (derivative-on-measurement already via errRateFilt)
        double p = kp * error;
        double d = kd * (errRateFilt);

        // kS with optional near-center boost (same blend style as your Basic)
        double ksNearBlend = 1.0 - Range.clip(absErr / Math.max(1e-6, ksNearRange), 0.0, 1.0);
        double ksTotal = ks + (ksNear * ksNearBlend);
        double ff = Math.signum(error) * ksTotal;

        double raw = SERVO_SIGN * (p + d + ff);

        // clamp max
        raw = Range.clip(raw, -MAX_POWER, MAX_POWER);

        // min power when moving
        if (Math.abs(raw) > 1e-6 && Math.abs(raw) < MIN_POWER) {
            raw = Math.signum(raw) * MIN_POWER;
        }

        // slew limit
        double maxDelta = Math.abs(SLEW_POWER_PER_SEC) * dt;
        out = slewLimit(out, raw, maxDelta);

        // soft limit gate (relative)
        if (ENABLE_SOFT_LIMIT) {
            out = gateAtRelLimits(out, turretRelRad);
        }

        turretServo.setPower(out);

        // debug
        lastErrDeg = error;
        lastErrRateDegPerSec = errRateFilt;

        return false;
    }

    // ---------------- Soft limit gate (relative angle) ----------------
    private static double gateAtRelLimits(double cmd, double turretRelRad) {
        double relDeg = Math.toDegrees(turretRelRad);

        if (relDeg >= MAX_REL_ANGLE_DEG) return Math.min(cmd, 0.0);
        if (relDeg <= -MAX_REL_ANGLE_DEG) return Math.max(cmd, 0.0);

        if (relDeg > 0 && relDeg >= (MAX_REL_ANGLE_DEG - LIMIT_CUSHION_DEG) && cmd > 0) return 0.0;
        if (relDeg < 0 && relDeg <= (-MAX_REL_ANGLE_DEG + LIMIT_CUSHION_DEG) && cmd < 0) return 0.0;

        return cmd;
    }

    // ---------------- Helpers ----------------
    private void resetFilters() {
        filtInit = false;
        errFilt = 0.0;
        errRateFilt = 0.0;
        lastErrFilt = 0.0;
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

    private static double wrapAngle(double rad) {
        while (rad <= -Math.PI) rad += 2.0 * Math.PI;
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        return rad;
    }

    // ---------------- Dashboard-friendly getters ----------------
    public double getLastErrDeg() { return lastErrDeg; }
    public double getLastErrRateDegPerSec() { return lastErrRateDegPerSec; }
    public double getLastOut() { return out; }

    public double getLastOdomErrDeg() { return lastOdomErrDeg; }
    public double getLastLLErrDeg() { return lastLLErrDeg; }
    public double getTurretAbsDeg() { return Math.toDegrees(getTurretAbsYawRad()); }
    //public double getTurretRelDeg(double drivetrainYawRad) { return getTurretRelDeg(drivetrainYawRad); }
}