package org.firstinspires.ftc.teamcode.A_Regional;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class C_Intake {
    private DcMotorEx intake;

    private double intake_power_intake = .9, intake_power_outtake = .6;

    public C_Intake(DcMotorEx motor){
        intake = motor;

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intake() {
        intake.setPower(intake_power_intake);
    }

    public void outtake() {
        intake.setPower(intake_power_outtake);

    }
    public Action intakes() {
        return packet -> {
            intake();
            return true;
        };
    }
    public Action stop() {
        return packet -> {
            deactivate();
            return true;
        };
    }
    public void properOuttake() {
        intake.setPower(-intake_power_outtake);

    }
    public void deactivate() {
        intake.setPower(0);
    }
}