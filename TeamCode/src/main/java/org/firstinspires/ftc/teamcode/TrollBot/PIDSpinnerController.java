package org.firstinspires.ftc.teamcode.TrollBot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDSpinnerController {

        // -------- Tunables (Dashboard) --------
        public static double kP = 0.0015;

        // Need to tune in lab
        public static double kI = 0.000001;
        public static double kD = 0.000015;


        // We can adjust these values based on if we need to use the max power
        public static double MIN_OUTPUT = -1.0;
        public static double MAX_OUTPUT =  1.0;

        public static double MAX_INTEGRAL = 0.4; // I saw this online so we keeping it
        public static double ERROR_TOLERANCE = 1.2; // This term tracks the allowed tolerance in the arm's movement/target pos




        // GoBilda 3.7:1 1620 rmp -> 103.8 PPR -> 415.2 ticks per rev (this is from goBilda)
        public static double TICKS_PER_REVOLUTION = 415.2;



        private double integral = 0.0;
        private double prevError = 0.0;
        private boolean first = true;

        public void reset() {
            integral = 0.0;
            prevError = 0.0;
            first = true;
        }

        public double update(double targetRmp, double currentRpm, double dtSeconds){
            if (dtSeconds<=0){
                dtSeconds = 1e-3;
            }

            double error = targetRmp - currentRpm;

            double e;
            if (Math.abs(error) < ERROR_TOLERANCE) {
                // If within tolerance, treat as exactly on target
                e = 0.0;
            } else {
                e = error;
            }

            // Integral with anti-windup limit
            integral += e * dtSeconds;
            integral = inRangeArm(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

            // Derivative for error
            double derivative = first ? 0.0 : (e - prevError) / dtSeconds;
            prevError = e;
            first = false;

            // Add all the P I D values for output
            double pTerm = kP * e;
            double iTerm = kI * integral;
            double dTerm = kD * derivative;

            double outputPower = pTerm + iTerm + dTerm;

            // Limit power to motor range
            outputPower = inRangeArm(outputPower, MIN_OUTPUT, MAX_OUTPUT);

            return outputPower;
        }


        // returns power set to motor based on the parameters
        public double updateArm(double targetPos, double currentPos, double dtSeconds) {
            if (dtSeconds <= 0){
                dtSeconds = 1e-3;
            }

            // Raw position error
            double error = targetPos - currentPos;

            // Effective error with tolerance
            // We use this for calculating power
            double e;
            if (Math.abs(error) < ERROR_TOLERANCE) {
                // If within tolerance, treat as exactly on target
                e = 0.0;
            } else {
                e = error;
            }

            // Integral with anti-windup limit
            integral += e * dtSeconds;
            integral = inRangeArm(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

            // Derivative for error
            double derivative = first ? 0.0 : (e - prevError) / dtSeconds;
            prevError = e;
            first = false;

            // Add all the P I D and F values for output
            double pTerm = kP * e;
            double iTerm = kI * integral;
            double dTerm = kD * derivative;

            double out = pTerm + iTerm + dTerm;

            // Limit power to motor range
            out = inRangeArm(out, MIN_OUTPUT, MAX_OUTPUT);

            return out;
        }

        private static double inRangeArm(double v, double lo, double hi) {
            return Math.max(lo, Math.min(hi, v));
        }
    }
