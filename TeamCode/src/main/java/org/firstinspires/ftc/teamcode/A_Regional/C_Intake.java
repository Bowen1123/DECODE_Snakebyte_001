package org.firstinspires.ftc.teamcode.A_Regional;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class C_Intake {
    private final DcMotorEx intake;

    // Keep your defaults, but make them configurable via setters if needed
    private double intakePower = 0.95;
    private double outtakePower = 0.55;

    public C_Intake(DcMotorEx motor) {
        intake = motor;
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        deactivate();
    }

    /** Pull in game piece. */
    public void intake() {
        intake.setPower(intakePower);
    }

    /** Push out (same direction as your original outtake()). */
    public void outtake() {
        intake.setPower(outtakePower);
    }

    /** Push out opposite direction (your original properOuttake()). */
    public void properOuttake() {
        intake.setPower(-outtakePower);
    }

    public void deactivate() {
        intake.setPower(0.0);
    }

    // Optional setters (nice for tuning quickly)
    public void setIntakePower(double p) { intakePower = p; }
    public void setOuttakePower(double p) { outtakePower = p; }

    public double getIntakePower() { return intakePower; }
    public double getOuttakePower() { return outtakePower; }
}