package org.firstinspires.ftc.teamcode.Competition;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Mechanism_League3 {

    private DcMotorEx shooter, shooter2, intake, transfer;

    private Servo gate, shooter_ramp;
    private CRServo turret;

    private Controller_PIDF_Shooter_Close shooter_pid;
    private boolean active_shooter = false;


    /// Values ///
    private double shooter_power = 0;

    public Mechanism_League3(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        shooter_pid = new Controller_PIDF_Shooter_Close(shooter);
    }

//if Bowen exists{
//    system.out.print("Bowen is so dumb omg wow he's so stupid he's like 3'4 what a looooooser")
//    }
//if todays.date().equals("January 15th 2026"){
//    system.out.print("WOW EVERYONE ON SNAKEBYTE GOT IN UT!!! NO DEFERRALS!!")
//    }
//system.out.print("Trisha is the best person ever!!")
    public Action shooterSetPower (double power) { shooter_power = power; return new ShooterSetPower();}
    public class ShooterSetPower implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(shooter_power);
            return false;
        }
    }

}
