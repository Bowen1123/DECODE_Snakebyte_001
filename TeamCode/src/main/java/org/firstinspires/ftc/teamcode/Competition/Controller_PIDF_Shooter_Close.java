package org.firstinspires.ftc.teamcode.Competition;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * PIDF controller for a flywheel / shooter:
 * - Controls RPM using encoder ticks (no getVelocity()).
 * - Uses System.currentTimeMillis() for timing (ms).
 */
@Config
public class Controller_PIDF_Shooter_Close {

    // ===== Dashboard-tunable PIDF coefficients =====
    /** Proportional gain: how strongly we react to RPM error. */
    public static double kP = 0.1;

    /** Integral gain: fixes small steady-state errors (usually small or zero). */
    public static double kI = 0;

    /** Derivative gain: reacts to change in error, helps damp oscillations. */
    public static double kD = 0; // 0.0000018

    /**
     * Feedforward gain:
     * maps targetRPM -> base power (open-loop guess).
     * power_ff ≈ kF * targetRPM
     */
    public static double kF = 0.00026;

    // ===== General config (also tunable on Dashboard) =====
    /** Encoder ticks per motor revolution. */
    public static double TICKS_PER_REV = 28.0;

    /** Maximum RPM for this motor (GoBILDA 6000 RPM). */
    public static double MAX_RPM = 6000.0;

    /** Minimum RPM (we’ll clamp here so you don’t go negative). */
    public static double MIN_RPM = 0.0;

    /** How much we change target RPM per button press in test TeleOp. */
    public static double RPM_STEP = 250.0;

    /** Deadband around target RPM for "close enough". */
    public static double RPM_TOLERANCE = 50;

    /** Limit on integral term to avoid crazy wind-up. */
    public static double MAX_INTEGRAL = 5000.0;

    // ===== Internal state =====
    private DcMotorEx motor;

    private double targetRpm = 0.0;
    private double currentRpm = 0.0;

    private double lastError = 0.0;
    private double integral = 0.0;

    private int lastPosition = 0;
    private long lastTimeMillis = 0L;

    private boolean atTarget = false;
    private double lastOutputPower = 0.0;

    public Controller_PIDF_Shooter_Close(DcMotorEx motor) {
        this.motor = motor;

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lastPosition = motor.getCurrentPosition();
        lastTimeMillis = System.currentTimeMillis();
    }

    /**
     * Call this every loop (~20ms) to update control.
     * It:
     *  1. Computes current RPM from encoder ticks + time.
     *  2. Runs PIDF.
     *  3. Applies power to the motor.
     *  4. Returns the power that was applied.
     */
    public static long MIN_DT_MILLIS = 69;   // don’t recompute velocity faster than this
    public static double RPM_ALPHA = 0.92;    // 0 = very smooth -> 1 = no smoothing

    public double update() {
        long now = System.currentTimeMillis();
        long dtMillis = now - lastTimeMillis;

        if (dtMillis < MIN_DT_MILLIS) {
            // Not enough time passed; just re-apply last power and return.
            motor.setPower(lastOutputPower);
            return lastOutputPower;
        }

        double dt = dtMillis / 1000.0;

        int position = motor.getCurrentPosition();
        int deltaTicks = position - lastPosition;

        // Ticks -> revs -> rev/s -> RPM
        double revs = deltaTicks / TICKS_PER_REV;
        double revsPerSec = revs / dt;
        double measuredRpm = revsPerSec * 60.0;

        // Exponential smoothing to reduce noise
        currentRpm = RPM_ALPHA * currentRpm + (1.0 - RPM_ALPHA) * measuredRpm;

        lastPosition = position;
        lastTimeMillis = now;

        // ----- PIDF control -----

        // Clamp target RPM
        targetRpm = clamp(targetRpm, MIN_RPM, MAX_RPM);

        double error = targetRpm - currentRpm;

        // Deadband: inside tolerance, treat as zero error
        if (Math.abs(error) <= RPM_TOLERANCE) {
            error = 0.0;
            atTarget = true;
        } else {
            atTarget = false;
        }

        // Integral (still zero for now, but we keep the logic)
        integral += error * dt;
        integral = clamp(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double pTerm = kP * error;
        double iTerm = kI * integral;
        double dTerm = kD * derivative;
        double fTerm = kF * targetRpm;

        double output = pTerm + iTerm + dTerm + fTerm;

        output = clamp(output, 0.0, 1.0);
        // motor.setPower(output);
        lastOutputPower = output;

        return output;
    }

    // ===== Helper getters/setters =====

    public void setTargetRpm(double rpm) {
        targetRpm = clamp(rpm, MIN_RPM, MAX_RPM);
    }

    public DcMotorEx getMotor(){
        return motor;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getCurrentRpm() {
        return currentRpm;
    }

    public double getLastOutputPower() {
        return lastOutputPower;
    }

    public boolean isAtTarget() {
        return atTarget;
    }

    // ===== Utility =====

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}