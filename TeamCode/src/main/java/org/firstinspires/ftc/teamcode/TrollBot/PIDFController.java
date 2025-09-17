package org.firstinspires.ftc.teamcode.TrollBot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDFController {

    // -------- Tunables (Dashboard) --------
    public static double kP = 0.015;

    // Need to tune in lab
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;


    // We can adjust these values based on if we need to use the max power
    public static double MIN_OUTPUT = -1.0;
    public static double MAX_OUTPUT =  1.0;

    public static double MAX_INTEGRAL = 0.4; // I saw this online so we keeping it
    public static double ERROR_DEADBAND = 0.0; // This term tracks the allowed tolerance in the arm's movement/target pos

    private double integral = 0.0;
    private double prevError = 0.0;
    private boolean first = true;

    public void reset() {
        integral = 0.0;
        prevError = 0.0;
        first = true;
    }

    // returns power set to motor based on the parameters
    public double update(double targetPos, double currentPos, double dtSeconds) {
        if (dtSeconds <= 0) dtSeconds = 1e-3;

        // Raw position error
        double error = targetPos - currentPos;

        // Effective error with tolerance
        // We use this for calculating power
        double e;
        if (Math.abs(error) < ERROR_DEADBAND) {
            // If within tolerance, treat as exactly on target
            e = 0.0;
        } else {
            e = error;
        }

        // Integral with anti-windup limit
        integral += e * dtSeconds;
        integral = inRange(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

        // Derivative for error
        double derivative = first ? 0.0 : (e - prevError) / dtSeconds;
        prevError = e;
        first = false;

        // Add all the P I D and F values for output
        double pTerm = kP * e;
        double iTerm = kI * integral;
        double dTerm = kD * derivative;
        double fTerm = kF;

        double out = pTerm + iTerm + dTerm + fTerm;

        // Limit power to motor range
        out = inRange(out, MIN_OUTPUT, MAX_OUTPUT);

        // Anti-windup helper: bleed integral if saturated and still pushing further
        // Idk if this is acc needed
        /*if ((out >= MAX_OUTPUT && e > 0) || (out <= MIN_OUTPUT && e < 0)) {
            integral *= 0.98;
        }*/

        return out;
    }

    private static double inRange(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
