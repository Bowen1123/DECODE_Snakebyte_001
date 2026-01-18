package org.firstinspires.ftc.teamcode.Competition;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Mechanism_League3 {

    private DcMotorEx topShooter, bottomShooter, intake, transfer;

    private Servo transferGate, shooter_ramp;
    private CRServo turret;

    private Controller_PIDF_Shooter_Close shooter_pid;
    private boolean active_shooter = false;


    /// Values ///
    private double shooter_power = 0;

    public Mechanism_League3(HardwareMap hardwareMap){
        topShooter = hardwareMap.get(DcMotorEx.class, "topShooter");
        bottomShooter = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        topShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        transferGate = hardwareMap.get(Servo.class, "transferGate");

        shooter_pid = new Controller_PIDF_Shooter_Close(topShooter);
    }
    ///  Preset Values ///
    double intakeInPower = 0.95, transferInPower = .9;
    double gateOpenPos = .25, gateClosePos = .55;

    public Action stopHalf() { return new StopHalf();}
    public Action allStop() { return new AllStop(); }
    public Action gateOpen() { return new GateOpen(); }
    public Action gateClose() { return new GateClose(); }
    public Action transferIn() {return new TransferIn();}
    public Action intakeSlow() { return new IntakeSlow(); }
    public Action transferOut() {return new TransferOut();}
    public Action stopTransfer() {return new StopTransfer(); }
    public Action intakeIn() {return new IntakeIn();}
    public Action stopIntake() {return new StopIntake(); }
    public Action shooterSetPower (double power) { shooter_power = power; return new ShooterSetPower();}
    public Action stopShooter () {return new StopShooter(); }

    public class StopHalf implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(0);
            intake.setPower(0);
            return false;
        }
    }
    public class AllStop implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            topShooter.setPower(0);
            bottomShooter.setPower(0);
            transfer.setPower(0);
            intake.setPower(0);
            return false;
        }
    }
    public class GateOpen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transferGate.setPosition(gateOpenPos);
            return false;
        }
    }
    public class GateClose implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transferGate.setPosition(gateClosePos);
            return false;
        }
    }

    public class TransferIn implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(transferInPower);
            return false;
        }
    }

    public class TransferOut implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(-transferInPower);
            return false;
        }
    }

    public class StopTransfer implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(0);
            return false;
        }
    }

    public class IntakeIn implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(intakeInPower);
            return false;
        }
    }

    public class IntakeSlow implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(intakeInPower);
            return false;
        }
    }

    public class StopIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(0);
            return false;
        }
    }

    public class ShooterSetPower implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            topShooter.setPower(shooter_power);
            bottomShooter.setPower(shooter_power);
            return false;
        }
    }

    public class StopShooter implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            topShooter.setPower(0);
            bottomShooter.setPower(0);
            return false;
        }
    }

}
