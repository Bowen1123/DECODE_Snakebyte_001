package org.firstinspires.ftc.teamcode.TrollBot;

public class PIDF_SpinnerController {
    public double kP, kI, kD, kF;

    private double targetRpm = 0.0;
    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastTime = -1.0;

    private double integralLimit = Double.POSITIVE_INFINITY;

    public PIDF_SpinnerController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /** Optional: limit the integral windup */
    public void setIntegralLimit(double limit) {
        integralLimit = Math.abs(limit);
    }

    public void setTargetRpm(double targetRpm) {
        targetRpm = targetRpm;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void reset() { // Reset stuff when parameters change
        integral = 0.0;
        lastError = 0.0;
        lastTime = -1.0;
    }

    /**
     * @param currentRpm    measured RPM of the motor
     * @param currentTimeS  current time in seconds (e.g. runtime.seconds())
     * @return power output (usually clip to [-1, 1] before sending to motor)
     */
    public double update(double currentRpm, double currentTimeS) {
        if (lastTime < 0) {
            // First run: just initialize time and error
            lastTime = currentTimeS;
            lastError = targetRpm - currentRpm;
            return 0.0;
        }

        double dt = currentTimeS - lastTime;
        if (dt <= 0) {
            // Avoid weird dt
            return 0.0;
        }

        double error = targetRpm - currentRpm;

        // Integral with anti-windup
        integral += error * dt;
        if (Math.abs(integral) > integralLimit) {
            /// Signum converts + to 1 && - to -1
            integral = Math.signum(integral) * integralLimit;
        }

        double derivative = (error - lastError) / dt;

        double pTerm = kP * error;
        double iTerm = kI * integral;
        double dTerm = kD * derivative;
        double fTerm = kF * targetRpm;  // basic feedforward on RPM

        double output = pTerm + iTerm + dTerm + fTerm;

        lastError = error;
        lastTime = currentTimeS;

        return output;
    }
}