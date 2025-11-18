package org.firstinspires.ftc.teamcode.TrollBot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PID_SpinnerController_Basic {

    // -------- Tunable Gains --------
    public double kP, kI, kD;

    // Output clamp
    public double MIN_OUTPUT = -1.0;
    public double MAX_OUTPUT =  1.0;

    // Integral clamp / deadband
    public double MAX_INTEGRAL = 0.5;
    public double ERROR_DEADBAND_RPM = 5.0;

    private double integral = 0.0;
    private double prevError = 0.0;
    private boolean first = true;

    public PID_SpinnerController_Basic(double kP, double kI, double kD) {
        setGains(kP, kI, kD);
    }

    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void reset() {
        integral = 0.0;
        prevError = 0.0;
        first = true;
    }

    public double update(double targetRpm, double currentRpm, double dtSeconds) {
        if (dtSeconds <= 0) dtSeconds = 1e-3;

        double rawError = targetRpm - currentRpm;

        // Apply deadband to avoid twitch near setpoint
        double e = (Math.abs(rawError) < ERROR_DEADBAND_RPM) ? 0.0 : rawError;

        // Integrate (with clamp)
        integral += e * dtSeconds;
        integral = clamp(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

        // Derivative
        double derivative = first ? 0.0 : (e - prevError) / dtSeconds;
        prevError = e;
        first = false;

        // PID only (no feedforward)
        double pTerm = kP * e;
        double iTerm = kI * integral;
        double dTerm = kD * derivative;

        double out = pTerm + iTerm + dTerm;

        // Clamp to motor range
        out = clamp(out, MIN_OUTPUT, MAX_OUTPUT);

        // Anti-windup bleed
        if ((out >= MAX_OUTPUT && e > 0) || (out <= MIN_OUTPUT && e < 0)) {
            integral *= 0.98;
        }

        return out;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}