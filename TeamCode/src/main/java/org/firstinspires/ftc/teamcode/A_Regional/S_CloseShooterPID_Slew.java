package org.firstinspires.ftc.teamcode.A_Regional;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class S_CloseShooterPID_Slew {

    // ---- Fixed system constants ----
    public static double ENCODER_TICKS_PER_REV = 28.0;
    public static double FLYWHEEL_RPM_PER_MOTOR_RPM = 2;

    // ---- Dashboard tunables ----
    public static double CFG_kP = 0.075;
    public static double CFG_kI = 0.00001;
    public static double CFG_kD = 0.0001;
    public static double CFG_kF = 0.025;

    public static double CFG_minPower = 0.15;
    public static double CFG_maxPower = 1.0;

    public static double CFG_maxPowerDeltaPerSec = 21.5;

    // NEW
    public static double CFG_rpmTolerance = 100.0;

    public static double CFG_maxTargetFlywheelRPM = 12000;

    // ---- Stored values ----
    private double kP, kI, kD, kF;
    private double minPower, maxPower;
    private double maxPowerDeltaPerSec;
    private double rpmTolerance;

    private double targetFlywheelRPM = 0.0;

    // ---- Internal state ----
    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastFlywheelRPM = 0.0;
    private double lastPowerCmd = 0.0;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTimeSec = 0.0;

    public S_CloseShooterPID_Slew() {
        syncFromDashboard();
        reset();
    }

    public S_CloseShooterPID_Slew(double kP, double kI, double kD, double kF) {
        setGains(kP, kI, kD, kF);
        setOutputLimits(0.0, 1.0);
        maxPowerDeltaPerSec = CFG_maxPowerDeltaPerSec;
        rpmTolerance = CFG_rpmTolerance;
        reset();
    }

    /** Pull @Config values */
    public void syncFromDashboard() {
        setGains(CFG_kP, CFG_kI, CFG_kD, CFG_kF);
        setOutputLimits(CFG_minPower, CFG_maxPower);
        maxPowerDeltaPerSec = Math.max(0.0, CFG_maxPowerDeltaPerSec);
        rpmTolerance = Math.max(0.0, CFG_rpmTolerance);
    }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        lastFlywheelRPM = 0.0;
        lastPowerCmd = 0.0;
        timer.reset();
        lastTimeSec = 0.0;
    }

    // ---- Setters ----
    public void setGains(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void setOutputLimits(double min, double max) {
        minPower = Range.clip(min, 0.0, 1.0);
        maxPower = Range.clip(max, 0.0, 1.0);
        if (minPower > maxPower) {
            double t = minPower;
            minPower = maxPower;
            maxPower = t;
        }
    }

    public void setTargetFlywheelRPM(double rpm) {
        targetFlywheelRPM = Math.max(0.0, rpm);
    }

    public void setMaxPowerDeltaPerSec(double v) {
        maxPowerDeltaPerSec = Math.max(0.0, v);
    }

    public void setRpmTolerance(double tol) {
        rpmTolerance = Math.max(0.0, tol);
    }

    // ---- Restored + New Getters ----
    public double getkP() { return kP; }
    public double getkI() { return kI; }
    public double getkD() { return kD; }
    public double getkF() { return kF; }

    public double getMinPower() { return minPower; }
    public double getMaxPower() { return maxPower; }

    public double getMaxPowerDeltaPerSec() { return maxPowerDeltaPerSec; }

    public double getTargetFlywheelRPM() { return targetFlywheelRPM; }

    public double getRpmTolerance() { return rpmTolerance; }

    public double getLastFlywheelRPM() { return lastFlywheelRPM; }

    public boolean isAtTarget() {
        return Math.abs(targetFlywheelRPM - lastFlywheelRPM) <= rpmTolerance;
    }

    // ---- Slew limiter ----
    private double applySlewRateLimit(double desiredPower, double dt) {
        if (maxPowerDeltaPerSec <= 0.0) return desiredPower;

        double maxDeltaThisCycle = maxPowerDeltaPerSec * dt;
        double delta = desiredPower - lastPowerCmd;

        if (delta > maxDeltaThisCycle) desiredPower = lastPowerCmd + maxDeltaThisCycle;
        else if (delta < -maxDeltaThisCycle) desiredPower = lastPowerCmd - maxDeltaThisCycle;

        return desiredPower;
    }

    public double update(double motorVelocityTicksPerSec) {
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

    public static double motorTicksPerSecToFlywheelRPM(double motorVelocityTicksPerSec) {
        double motorRPM = (motorVelocityTicksPerSec * 60.0) / ENCODER_TICKS_PER_REV;
        return motorRPM * FLYWHEEL_RPM_PER_MOTOR_RPM;
    }
}
