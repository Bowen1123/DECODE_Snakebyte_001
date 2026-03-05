package org.firstinspires.ftc.teamcode.A_Regional;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * C_ShooterTurret
 *
 * Literal combination of:
 *  - C_Shooter (flywheels + ramp + internal Slew PIDF)
 *  - D_BasicTurret (Limelight tracking PD + kS + filtering + slew)
 *
 * Logic is preserved; only method names that would collide were disambiguated:
 *  - shooterStop() for shooter motors
 *  - turretStop() for turret servo
 */
@Config
public class C_ShooterTurret {
    // ---------------- Manual shooter target (Dashboard) ----------------
    public static boolean USE_MANUAL_SHOOTER_TARGET = false;

    // Used when USE_MANUAL_SHOOTER_TARGET = true
    public static double MANUAL_TARGET_RPM = 2400;
    public static double MANUAL_RAMP_POS = 0.38;
    private double turretCmd = 0;

    // ============================================================
    // ======================= SHOOTER ============================
    // ============================================================

    private Servo leftLED, rightLED;

    // ---------------- Ramp bounds (your values) ----------------
    public static double RAMP_MIN = 0.38; // all the way down
    public static double RAMP_MAX = 0.80;

    // ---------------- Adaptive equations clamps ----------------
    public static double ADAPTIVE_RPM_MIN = 2000;
    public static double ADAPTIVE_RPM_MAX = 7000;

    // ---------------- Slew PIDF constants ----------------
    public static double ENCODER_TICKS_PER_REV = 28.0;
    public static double FLYWHEEL_RPM_PER_MOTOR_RPM = 2.0;

    // Dashboard tunables
    public static double CFG_kP = 0.0076;
    public static double CFG_kI = 0.000001;
    public static double CFG_kD = 0.0000025;
    public static double CFG_kF = 0.000001;

    public static double CFG_minPower = 0.07;
    public static double CFG_maxPower = 1.0;

    public static double CFG_maxPowerDeltaPerSec = 100;
    public static double CFG_rpmTolerance = 67;

    public static double CFG_maxTargetFlywheelRPM = 12000;

    double testTargetRPM = 0;
    double testTargetRampPos = 0;

    // ---------------- Shooter Hardware ----------------
    private final DcMotorEx topShooter, bottomShooter;
    private final Servo shooterRamp;
    private final DcMotorEx encoderMotor;

    // ---------------- Shooter State ----------------
    private boolean shooterEnabled = false;
    private double distanceIn = 0.0;

    // Internal controller
    private final SlewPidf pid = new SlewPidf();

    // ============================================================
    // ======================== TURRET ============================
    // ============================================================

    // ---------------- Limelight ----------------
    public static int POLL_RATE_HZ = 100;
    public static int PIPELINE_INDEX = 0;

    // ---------------- Distance geometry ----------------
    public static double MOUNT_ANGLE_DEG = 11.0;
    public static double CAM_HEIGHT_IN   = 11.0;
    public static double POI_HEIGHT_IN   = 32.0;

    // ---------------- Behavior ----------------
    public static double SERVO_SIGN = +1.0;
    public static double DEADBAND_DEG = 3;

    // ---------------- Controller (PD + kS) ----------------
    public static double KP = 0.01295;
    public static double KD = 0.0033;

    public static double KS = 0.14;

    public static double KS_NEAR = 0.165;
    public static double KS_NEAR_RANGE_DEG = 7;

    // ---------------- Filtering ----------------
    public static double TX_LP_ALPHA = .55;
    public static double RATE_LP_ALPHA = .6;

    // ---------------- Output limits ----------------
    public static double MAX_POWER = 0.45;
    public static double MIN_POWER = .182;
    public static double SLEW_POWER_PER_SEC = 100;

    // ---------------- Target handling ----------------
    public static double TARGET_HOLD_SEC = 0.02;
    public static double LOST_OUTPUT_DECAY_PER_SEC = 100;

    // ---------------- Timing clamps ----------------
    public static double DT_MIN = 1e-3;
    public static double DT_MAX = 0.08;

    // ---------------- Turret Hardware ----------------
    private final Limelight3A ll;
    private final CRServo turretServo;

    private boolean trackingEnabled = false;

    // Latest Limelight readings
    private boolean hasValidTarget = false;
    private double txDeg = 0.0;
    private double tyDeg = 0.0;

    // Time
    private final ElapsedTime turretLoopTimer = new ElapsedTime();
    private double tSec = 0.0;
    private double lastSeenSec = -999.0;

    // Filtered signals
    private boolean filtInit = false;
    private double txFilt = 0.0;
    private double txRateFilt = 0.0;
    private double lastTxFilt = 0.0;

    // Output state (slew/hold)
    private double out = 0.0;

    // ============================================================
    // ===================== CONSTRUCTOR ==========================
    // ============================================================

    public C_ShooterTurret(DcMotorEx top,
                           DcMotorEx bottom,
                           Servo ramp,
                           Servo left,
                           Servo right,
                           Limelight3A limelight,
                           CRServo turret) {

        // ---- Shooter init (same as your C_Shooter constructor) ----
        this.topShooter = top;
        this.bottomShooter = bottom;
        this.shooterRamp = ramp;

        this.encoderMotor = bottomShooter;

        topShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        topShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        bottomShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLED = left;
        rightLED = right;
        leftLED.setPosition(.71);
        rightLED.setPosition(.71);

        pid.reset();
        shooterStop();
        setRampPosition(RAMP_MIN);

        // ---- Turret init (same as your D_BasicTurret init/start flow) ----
        this.ll = limelight;
        this.turretServo = turret;

        ll.setPollRateHz(POLL_RATE_HZ);
        ll.pipelineSwitch(PIPELINE_INDEX);
        ll.start();

        turretLoopTimer.reset();
        tSec = 0.0;
        resetTurretFilters();
        out = 0.0;
        lastSeenSec = -999.0;

        setTrackingEnabled(false);
        setShooterEnabled(false);
    }

    // ============================================================
    // ================== SHOOTER: ADAPTIVE =======================
    // ============================================================

    /** distance -> target flywheel RPM */
    public double getAdaptiveTargetFlywheelRPM(double distance_INCH) {
        double rpm;
        if (distance_INCH < 110) {
            rpm = (14.75571 * distance_INCH) + 1949.97742;
        } else if (distance_INCH >= 110) {
            rpm = 3990;

        } else if (distance_INCH >= 120) {
            rpm = 4100;
        } else {
            rpm = 2400;
        }
        rpm = Range.clip(rpm, ADAPTIVE_RPM_MIN, ADAPTIVE_RPM_MAX);
        rpm = Math.min(rpm, CFG_maxTargetFlywheelRPM);
        return rpm;
    }

    /** distance -> target ramp servo position */
    public double getAdaptiveTargetRampPos(double distance_INCH) {
        double pos;
        if (distance_INCH < 110) {
            pos = .4;
        } else if (distance_INCH >= 110) {
            pos = 0.734;
            // pos = Math.max(Math.min((-0.010863 * distance_INCH ) +2.01398, 0.78), 0.54);
        } else {
            pos = 0.38;
        }
        return Range.clip(pos, RAMP_MIN, RAMP_MAX);
    }
    ///  LED
    public void setLeftLEDPos(double pos){
        leftLED.setPosition(pos);
    }
    public void setPipelines(int pipeline){
        ll.pipelineSwitch(pipeline);
    }

    public void setRightLEDOis(double pos){
        rightLED.setPosition(pos);
    }
    public void setBothLEDPos(double pos){
        leftLED.setPosition(pos);
        rightLED.setPosition(pos);
    }
    // ============================================================
    // ===================== SHOOTER: API =========================
    // ============================================================

    public void syncFromDashboard() {
        pid.syncFromDashboard();
    }

    public void setShooterEnabled(boolean enabled) {
        if (this.shooterEnabled != enabled) {
            this.shooterEnabled = enabled;
            pid.reset();
            if (!enabled) shooterStop();
        }
    }

    public boolean isShooterEnabled() {
        return shooterEnabled;
    }

    public void setDistanceInches(double distanceIn) {
        this.distanceIn = Math.max(0.0, distanceIn);
    }

    public double getDistanceInches() {
        return distanceIn;
    }

    public void setRampPosition(double pos) {
        shooterRamp.setPosition(Range.clip(pos, RAMP_MIN, RAMP_MAX));
    }

    public void setManualRampPos (double pos){
        MANUAL_RAMP_POS = pos;
    }

    public double getRampPosition() {
        return shooterRamp.getPosition();
    }

    public double getMeasuredRPM() {
        return motorTicksPerSecToFlywheelRPM(encoderMotor.getVelocity());
    }

    public double getTargetRPM() {
        return pid.getTargetFlywheelRPM();
    }

    public boolean isAtTarget() {
        return pid.isAtTarget();
    }

    public void setTestTargetRPM(double rpm) {
        testTargetRPM = rpm;
    }

    public void setTestTargetRampPos(double pos) {
        testTargetRampPos = pos;
    }

    public Action updateAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shooterEnabled = true;
                distanceIn = 110;
                shooterUpdate();
                return true;
            }
        };
    }

    public Action ramp() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setTestTargetRampPos(.84);
                return false;
            }
        };
    }

    /**
     * Shooter main loop:
     * computes adaptive target RPM + ramp from current distance,
     * runs slew PIDF if enabled, writes motors.
     */
    public double shooterUpdate() {

        double targetRPM;
        double targetRamp;

        if (USE_MANUAL_SHOOTER_TARGET) {
            targetRPM = MANUAL_TARGET_RPM;
            targetRamp = MANUAL_RAMP_POS;
        } else {
            targetRPM = getAdaptiveTargetFlywheelRPM(distanceIn);
            targetRamp = getAdaptiveTargetRampPos(distanceIn);
        }

        pid.setTargetFlywheelRPM(targetRPM);
        setRampPosition(targetRamp);

        double powerCmd = 0.0;
        if (shooterEnabled && targetRPM > 0.0) {
            powerCmd = pid.update(encoderMotor.getVelocity());
        }

        topShooter.setPower(powerCmd);
        bottomShooter.setPower(powerCmd);
        return powerCmd;
    }

    /** Shooter stop (renamed to avoid collision with turret stop). */
    public void shooterStop() {
        topShooter.setPower(0.0);
        bottomShooter.setPower(0.0);
    }

    public void resetShooterController() {
        pid.reset();
    }

    // ============================================================
    // ============ SHOOTER: INTERNAL SLEW PIDF ====================
    // ============================================================

    private static class SlewPidf {
        private double kP, kI, kD, kF;
        private double minPower, maxPower;
        private double maxPowerDeltaPerSec;
        private double rpmTolerance;

        private double targetFlywheelRPM = 0.0;

        private double integral = 0.0;
        private double lastError = 0.0;
        private double lastFlywheelRPM = 0.0;
        private double lastPowerCmd = 0.0;

        private final ElapsedTime timer = new ElapsedTime();
        private double lastTimeSec = 0.0;

        private SlewPidf() {
            syncFromDashboard();
            reset();
        }

        private void syncFromDashboard() {
            setGains(CFG_kP, CFG_kI, CFG_kD, CFG_kF);
            setOutputLimits(CFG_minPower, CFG_maxPower);
            maxPowerDeltaPerSec = Math.max(0.0, CFG_maxPowerDeltaPerSec);
            rpmTolerance = Math.max(0.0, CFG_rpmTolerance);
        }

        private void reset() {
            integral = 0.0;
            lastError = 0.0;
            lastFlywheelRPM = 0.0;
            lastPowerCmd = 0.0;
            timer.reset();
            lastTimeSec = 0.0;
        }

        private void setGains(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }

        private void setOutputLimits(double min, double max) {
            minPower = Range.clip(min, 0.0, 1.0);
            maxPower = Range.clip(max, 0.0, 1.0);
            if (minPower > maxPower) {
                double t = minPower;
                minPower = maxPower;
                maxPower = t;
            }
        }

        private void setTargetFlywheelRPM(double rpm) {
            targetFlywheelRPM = Math.max(0.0, rpm);
        }

        private double getTargetFlywheelRPM() {
            return targetFlywheelRPM;
        }

        private boolean isAtTarget() {
            return Math.abs(targetFlywheelRPM - lastFlywheelRPM) <= rpmTolerance;
        }

        private double applySlewRateLimit(double desiredPower, double dt) {
            if (maxPowerDeltaPerSec <= 0.0) return desiredPower;

            double maxDeltaThisCycle = maxPowerDeltaPerSec * dt;
            double delta = desiredPower - lastPowerCmd;

            if (delta > maxDeltaThisCycle) desiredPower = lastPowerCmd + maxDeltaThisCycle;
            else if (delta < -maxDeltaThisCycle) desiredPower = lastPowerCmd - maxDeltaThisCycle;

            return desiredPower;
        }

        private double update(double motorVelocityTicksPerSec) {
            double now = timer.seconds();
            double dt = now - lastTimeSec;
            if (dt <= 1e-4) dt = 1e-4;
            lastTimeSec = now;

            double motorRPM = (motorVelocityTicksPerSec * 60.0) / ENCODER_TICKS_PER_REV;
            lastFlywheelRPM = motorRPM * FLYWHEEL_RPM_PER_MOTOR_RPM;

            double error = targetFlywheelRPM - lastFlywheelRPM;

            integral += error * dt;

            double derivative = (error - lastError) / dt;
            lastError = error;

            double ff = kF * targetFlywheelRPM;
            double pid = (kP * error) + (kI * integral) + (kD * derivative);

            double desiredPower = Range.clip(ff + pid, minPower, maxPower);
            double power = applySlewRateLimit(desiredPower, dt);

            final double SAT_EPS = 1e-3;
            if ((power >= maxPower - SAT_EPS && error > 0) ||
                    (power <= minPower + SAT_EPS && error < 0)) {
                integral -= error * dt;
            }

            lastPowerCmd = power;
            return power;
        }
    }

    public static double motorTicksPerSecToFlywheelRPM(double motorVelocityTicksPerSec) {
        double motorRPM = (motorVelocityTicksPerSec * 60.0) / ENCODER_TICKS_PER_REV;
        return motorRPM * FLYWHEEL_RPM_PER_MOTOR_RPM;
    }

    // ============================================================
    // ===================== TURRET: API ==========================
    // ============================================================

    public void setTrackingEnabled(boolean enabled) {
        if (enabled != trackingEnabled) {
            trackingEnabled = enabled;
            resetTurretFilters();
            out = 0.0;
            if (!trackingEnabled) turretStop();
        }
    }

    public boolean isTrackingEnabled() { return trackingEnabled; }

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

    public double getGroundDistanceInches() {
        if (!hasValidTarget) return Double.NaN;

        double deltaH = POI_HEIGHT_IN - CAM_HEIGHT_IN;
        double angleDeg = MOUNT_ANGLE_DEG + tyDeg;

        double tan = Math.tan(Math.toRadians(angleDeg));
        if (Math.abs(tan) < 1e-6) return Double.NaN;

        return deltaH / tan;
    }


    public void setTurretPower( double pow ){
        turretServo.setPower(Math.max(Math.min(pow, .45), .17));
    }


    /**
     * Turret main loop (same as your loop(), renamed to avoid confusion with shooterUpdate()).
     * Returns true when aimed.
     */
    public boolean turretLoop() {
        // dt
        double dt = turretLoopTimer.seconds();
        turretLoopTimer.reset();
        if (dt < DT_MIN) dt = DT_MIN;
        if (dt > DT_MAX) dt = DT_MAX;

        tSec += dt;

        if (!trackingEnabled) {
            turretStop();
            return false;
        }

        boolean recentlySeen = (tSec - lastSeenSec) <= TARGET_HOLD_SEC;

        if (hasValidTarget) {
            if (!filtInit) {
                txFilt = txDeg;
                lastTxFilt = txFilt;
                txRateFilt = 0.0;
                filtInit = true;
            } else {
                txFilt = txFilt + TX_LP_ALPHA * (txDeg - txFilt);

                double txRate = (txFilt - lastTxFilt) / dt;
                lastTxFilt = txFilt;

                txRateFilt = txRateFilt + RATE_LP_ALPHA * (txRate - txRateFilt);
            }
        }

        if (!recentlySeen || !filtInit) {
            out = decayTowardZero(out, LOST_OUTPUT_DECAY_PER_SEC * dt);
            turretServo.setPower(out);
            return false;
        }

        double error = -txFilt;
        double absErr = Math.abs(error);

        if (absErr <= DEADBAND_DEG) {
            out = 0.0;
            turretServo.setPower(0.0);
            return true;
        }

        double p = KP * error;
        double d = KD * (-txRateFilt);

        double ksNearBlend = 1.0 - Range.clip(absErr / Math.max(1e-6, KS_NEAR_RANGE_DEG), 0.0, 1.0);
        double ksTotal = KS + (KS_NEAR * ksNearBlend);
        double ff = Math.signum(error) * ksTotal;

        double raw = SERVO_SIGN * (p + d + ff);

        raw = Range.clip(raw, -MAX_POWER, MAX_POWER);

        if (Math.abs(raw) > 1e-6 && Math.abs(raw) < MIN_POWER) {
            raw = Math.signum(raw) * MIN_POWER;
        }

        double maxDelta = Math.abs(SLEW_POWER_PER_SEC) * dt;
        out = slewLimit(out, raw, maxDelta);

        turretCmd = out;
        turretServo.setPower(out);

        return false;
    }

    public double getTurretCmd (){
        return turretCmd;
    }

    public void setPipeline(int pipe) {
        ll.pipelineSwitch(pipe);
        PIPELINE_INDEX = pipe;
    }

    public int getPipeline() {
        return PIPELINE_INDEX;
    }

    /** Turret stop (renamed to avoid collision with shooterStop()). */
    public void turretStop() {
        turretServo.setPower(0.0);
        out = 0.0;
    }

    // ---------------- Turret helpers ----------------
    private void resetTurretFilters() {
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

    /** Convenience to stop everything. */
    public void stopAll() {
        setShooterEnabled(false);
        shooterStop();

        setTrackingEnabled(false);
        turretStop();
    }
}