package org.firstinspires.ftc.teamcode.A_Regional;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * C_Shooter
 * - Owns: top/bottom flywheel motors + ramp servo
 * - Contains: internal Slew PIDF controller (no separate class needed)
 * - Provides two "holder" methods for adaptive shooter:
 *      getAdaptiveTargetFlywheelRPM(distanceIn)
 *      getAdaptiveTargetRampPos(distanceIn)
 *
 * Typical loop usage:
 *   shooter.syncFromDashboard();          // optional live tuning
 *   shooter.setDistanceInches(distIn);
 *   shooter.setEnabled(enabled);
 *   shooter.update();                    // computes targets + runs PID + writes motors
 */
@Config
public class C_Shooter {

    private Servo leftLED, rightLED;
    // ---------------- Ramp bounds (your values) ----------------
    public static double RAMP_MIN = 0.38; // all the way down
    public static double RAMP_MAX = 0.80;

    // ---------------- Adaptive equations clamps ----------------
    public static double ADAPTIVE_RPM_MIN = 2000;
    public static double ADAPTIVE_RPM_MAX = 4000;

    // ---------------- Slew PIDF constants (moved IN here) ----------------
    // Fixed system constants
    public static double ENCODER_TICKS_PER_REV = 28.0;
    public static double FLYWHEEL_RPM_PER_MOTOR_RPM = 2.0;

    // Dashboard tunables (same names so you can tune in Dashboard under C_Shooter)
    public static double CFG_kP = 0.075;
    public static double CFG_kI = 0.00001;
    public static double CFG_kD = 0.0001;
    public static double CFG_kF = 0.025;

    public static double CFG_minPower = 0.15;
    public static double CFG_maxPower = 1.0;

    public static double CFG_maxPowerDeltaPerSec = 21.5;
    public static double CFG_rpmTolerance = 100.0;

    public static double CFG_maxTargetFlywheelRPM = 12000;
    double testTargetRPM = 0;
    double testTargetRampPos = 0;

    // ---------------- Hardware ----------------
    private final DcMotorEx topShooter, bottomShooter;
    private final Servo shooterRamp;

    // Which motor provides velocity feedback (matches your tuning: bottom)
    private final DcMotorEx encoderMotor;

    // ---------------- State ----------------
    private boolean enabled = false;
    private double distanceIn = 0.0;

    // Internal controller
    private final SlewPidf pid = new SlewPidf();

    public C_Shooter(DcMotorEx top, DcMotorEx bottom, Servo ramp, Servo left, Servo right) {
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
        stop();
        setRampPosition(RAMP_MIN);
    }

    // ============================================================
    // Holder Methods for Adaptive Shooter
    // ============================================================

    /** Holder method #1: distance -> target flywheel RPM */
    public double getAdaptiveTargetFlywheelRPM(double distance_INCH) {
        double rpm;
        if (distance_INCH < 105) {
            rpm = (distance_INCH * 8.54487) + 1704.53569; // your equation
        } else if (distance_INCH >= 105){
            rpm = 3125;
        } else {
            rpm = 2550;
        }
        //y=8.54487x+1704.53569
        rpm = Range.clip(rpm, ADAPTIVE_RPM_MIN, ADAPTIVE_RPM_MAX);
        rpm = Math.min(rpm, CFG_maxTargetFlywheelRPM);
        return rpm;
    }

    /** Holder method #2: distance -> target ramp servo position */
    public double getAdaptiveTargetRampPos(double distance_INCH) {
        double pos;
        if (distance_INCH < 105) {
            pos = .4;
        } else if (distance_INCH >= 105){
            pos = .78;
        } else {
            pos = 0.38;
        }

        return Range.clip(pos, RAMP_MIN, RAMP_MAX);
    }

    // ============================================================
    // Public API
    // ============================================================

    /** Optional live tuning: call in loop() if you want dashboard changes applied. */
    public void syncFromDashboard() {
        pid.syncFromDashboard();
    }

    public void setEnabled(boolean enabled) {
        if (this.enabled != enabled) {
            this.enabled = enabled;
            pid.reset();
            if (!enabled) stop();
        }
    }

    public boolean isEnabled() {
        return enabled;
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

    public void setTestTargetRPM(double rpm){
        testTargetRPM = rpm;
    }
    public void setTestTargetRampPos(double pos){
        testTargetRampPos = pos;
    }

    /**
     * Main loop call:
     * - computes adaptive target RPM + target ramp from current distance
     * - runs internal slew PIDF if enabled
     * - writes motor powers
     * Returns applied power command.
     */
    public double update() {
        double targetRPM = getAdaptiveTargetFlywheelRPM(distanceIn);
        double targetRamp = getAdaptiveTargetRampPos(distanceIn);

//        targetRamp = testTargetRampPos;
//        targetRPM = testTargetRPM;

        pid.setTargetFlywheelRPM(targetRPM);
        setRampPosition(targetRamp);

        double powerCmd = 0.0;
        if (enabled && targetRPM > 0.0) {
            powerCmd = pid.update(encoderMotor.getVelocity());
        }

        topShooter.setPower(powerCmd);
        bottomShooter.setPower(powerCmd);
        return powerCmd;
    }

    public void stop() {
        topShooter.setPower(0.0);
        bottomShooter.setPower(0.0);
    }

    public void resetController() {
        pid.reset();
    }

    // ============================================================
    // Internal Slew PIDF Controller (embedded)
    // ============================================================

    private static class SlewPidf {
        // Stored values (copied from config each sync)
        private double kP, kI, kD, kF;
        private double minPower, maxPower;
        private double maxPowerDeltaPerSec;
        private double rpmTolerance;

        private double targetFlywheelRPM = 0.0;

        // Internal state
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

        /** Pull @Config values from outer C_Shooter class */
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

        // Slew limiter
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

            // Convert to flywheel RPM
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

            // Anti-windup near saturation
            final double SAT_EPS = 1e-3;
            if ((power >= maxPower - SAT_EPS && error > 0) ||
                    (power <= minPower + SAT_EPS && error < 0)) {
                integral -= error * dt;
            }

            lastPowerCmd = power;
            return power;
        }
    }

    // ============================================================
    // Static helper (kept so your other code can use it too)
    // ============================================================

    public static double motorTicksPerSecToFlywheelRPM(double motorVelocityTicksPerSec) {
        double motorRPM = (motorVelocityTicksPerSec * 60.0) / ENCODER_TICKS_PER_REV;
        return motorRPM * FLYWHEEL_RPM_PER_MOTOR_RPM;
    }
}