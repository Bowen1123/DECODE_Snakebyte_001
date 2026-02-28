package org.firstinspires.ftc.teamcode.A_Regional;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class C_Transfer {
    private DcMotorEx transfer;
    private Servo transferGate;
    private RevColorSensorV3 color;
    private double gateOpen = .65, gateClose = 0.6;
    private double transfer_power_intake = .5, transfer_power_outtake = .9;

    public C_Transfer(DcMotorEx motor, Servo servo, RevColorSensorV3 sensor){
        transfer = motor;
        transferGate = servo;
        color = sensor;

        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    ///  Close gate and use smaller power
    public void intake() {
        transferGate.setPosition(gateClose);
        transfer.setPower(transfer_power_intake);
    }

    ///  Opens gate and use larger power
    public void outtake() {
        transferGate.setPosition(gateOpen);
        transfer.setPower(transfer_power_outtake); // negative if reversing
    }
    public void properOutake() {
        transferGate.setPosition(gateClose);
        transfer.setPower(-transfer_power_outtake); // negative if reversing
    }
    public Action intakes() {
        return packet -> {
            intake();     // run PID + write motors
            return true;  // keep running forever
        };
    }

    public Action outtakes() {
        return packet -> {
            outtake();     // run PID + write motors
            return true;  // keep running forever
        };
    }

    /// Stops stuff
    public void deactivate() {
        transfer.setPower(0.0);
        transferGate.setPosition(gateClose);
    }
}
