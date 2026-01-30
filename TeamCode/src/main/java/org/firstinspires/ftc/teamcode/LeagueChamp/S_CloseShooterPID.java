package org.firstinspires.ftc.teamcode.LeagueChamp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class S_CloseShooterPID {

    // ---- Fixed system constants ----
    public static double ENCODER_TICKS_PER_REV = 28.0;
    public static double FLYWHEEL_RPM_PER_MOTOR_RPM = 1.5;

    // ---- Dashboard tunables (edit live in FTC Dashboard) ----
    public static double CFG_kP = 0.0101;
    public static double CFG_kI = 0.00005;
    public static double CFG_kD = 0.0002;
    public static double CFG_kF = 0.00001;

    public static double CFG_minPower = 0.4;
    public static double CFG_maxPower = 1.0;

    // Safety cap for TeleOp targets
    public static double CFG_maxTargetFlywheelRPM = 6000;

    // ---- Stored controller values (instance owns these) ----
    private double kP, kI, kD, kF;
    private double minPower, maxPower;

    private double targetFlywheelRPM = 0.0;

    // ---- Internal state ----
    private double integral = 0.0;
    private double lastError = 0.0;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTimeSec = 0.0;

    public S_CloseShooterPID() {
        // Start from dashboard defaults
        syncFromDashboard();
        reset();
    }

    public S_CloseShooterPID(double kP, double kI, double kD, double kF) {
        setGains(kP, kI, kD, kF);
        setOutputLimits(0.0, 1.0);
        reset();
    }

    /** Pull current @Config values into the controller (call in TeleOp loop if you want live tuning). */
    public void syncFromDashboard() {
        setGains(CFG_kP, CFG_kI, CFG_kD, CFG_kF);
        setOutputLimits(CFG_minPower, CFG_maxPower);
    }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        timer.reset();
        lastTimeSec = 0.0;
    }

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

    public double getTargetFlywheelRPM() {
        return targetFlywheelRPM;
    }

    public double getkP() { return kP; }
    public double getkI() { return kI; }
    public double getkD() { return kD; }
    public double getkF() { return kF; }
    public double getMinPower() { return minPower; }
    public double getMaxPower() { return maxPower; }

    /**
     * Update controller based on topShooter encoder velocity.
     * @param motorVelocityTicksPerSec topShooter.getVelocity() (ticks/sec)
     * @return power command [minPower..maxPower]
     */
    public double update(double motorVelocityTicksPerSec) {
        double now = timer.seconds();
        double dt = now - lastTimeSec;
        if (dt <= 1e-4) dt = 1e-4;
        lastTimeSec = now;

        // motor ticks/sec -> motor RPM
        double motorRPM = (motorVelocityTicksPerSec * 60.0) / ENCODER_TICKS_PER_REV;

        // motor RPM -> flywheel RPM
        double flywheelRPM = motorRPM * FLYWHEEL_RPM_PER_MOTOR_RPM;

        double error = targetFlywheelRPM - flywheelRPM;

        // Integrate with light anti-windup (handled after clamp too)
        integral += error * dt;

        double derivative = (error - lastError) / dt;
        lastError = error;

        double ff = kF * targetFlywheelRPM;
        double pid = (kP * error) + (kI * integral) + (kD * derivative);

        double power = ff + pid;
        power = Range.clip(power, minPower, maxPower);

        // Anti-windup: if saturated and error pushes further into saturation, undo that integration step
        if ((power >= maxPower && error > 0) || (power <= minPower && error < 0)) {
            integral -= error * dt;
        }

        return power;
    }

    /** Helper for telemetry. */
    public static double motorTicksPerSecToFlywheelRPM(double motorVelocityTicksPerSec) {
        double motorRPM = (motorVelocityTicksPerSec * 60.0) / ENCODER_TICKS_PER_REV;
        return motorRPM * FLYWHEEL_RPM_PER_MOTOR_RPM;
    }
}