package org.firstinspires.ftc.teamcode.A_Regional;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class C_Transfer {
    private final DcMotorEx transfer;
    private final Servo transferGate;
    private final RevColorSensorV3 color; // may be null / unused

    private double gateOpen = 0.65;
    private double gateClose = 0.55;

    private double intakePower = 0.64;
    private double outtakePower = 0.92;

    public C_Transfer(DcMotorEx motor, Servo servo, RevColorSensorV3 sensor) {
        transfer = motor;
        transferGate = servo;
        color = sensor;

        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        deactivate();
    }

    /** Close gate and run smaller power to bring piece inward. */
    public void intake() {
        setGateClosed();
        transfer.setPower(intakePower);
    }

    /** Open gate and run larger power to feed outward (to shooter). */
    public void outtake() {
        setGateOpen();
        transfer.setPower(outtakePower);
    }

    /** Reverse outtake while keeping gate closed (your original properOutake()). */
    public void properOuttake() {
        setGateClosed();
        transfer.setPower(-outtakePower);
    }

    public void setGateOpen() {
        transferGate.setPosition(gateOpen);
    }

    public void setGateClosed() {
        transferGate.setPosition(gateClose);
    }

    /** Stops motor and closes gate. */
    public void deactivate() {
        transfer.setPower(0.0);
        setGateClosed();
    }

    // Optional setters/getters
    public void setGateOpenPos(double p) { gateOpen = p; }
    public void setGateClosePos(double p) { gateClose = p; }
    public void setIntakePower(double p) { intakePower = p; }
    public void setOuttakePower(double p) { outtakePower = p; }

    public double getGateOpenPos() { return gateOpen; }
    public double getGateClosePos() { return gateClose; }
    public double getIntakePower() { return intakePower; }
    public double getOuttakePower() { return outtakePower; }

    /** If you ever want to use it later */
    public RevColorSensorV3 getColorSensor() { return color; }
}