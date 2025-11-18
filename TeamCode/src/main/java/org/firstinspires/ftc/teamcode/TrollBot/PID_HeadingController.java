package org.firstinspires.ftc.teamcode.TrollBot;

public class PID_HeadingController {
    private double kp, ki, kd;
    private double integral;
    private double prevError;
    private boolean first = true;

    private double integralMin = -1e6, integralMax = 1e6;

    public PID_HeadingController(double kp, double ki, double kd) {
        setGains(kp, ki, kd);
    }

    public void setGains(double kp, double ki, double kd) {
        this.kp = kp; this.ki = ki; this.kd = kd;
    }

    public void setIntegralLimits(double min, double max) {
        this.integralMin = min; this.integralMax = max;
    }

    public void reset() {
        integral = 0.0;
        prevError = 0.0;
        first = true;
    }

    public double update(double target, double current) {
        double error = target - current;

        integral += error;
        if (integral > integralMax) {
            integral = integralMax;
        }
        if (integral < integralMin){
            integral = integralMin;
        }

        double deriv = first ? 0.0 : (error - prevError);

        double out = kp * error + ki * integral + kd * deriv;

        prevError = error;
        first = false;
        return out;
    }
}
