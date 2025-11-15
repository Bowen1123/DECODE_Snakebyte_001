package org.firstinspires.ftc.teamcode.Competition;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mechanism {

    private DcMotorEx shooter, intake, transfer;


    public Mechanism(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
    }


    public Action shooterPowerUp() { return new ShooterPowerUp(); }
    public Action shooterPowerDown() { return  new ShooterPowerDown(); }
    public Action intakeIn() { return new IntakeIn(); }
    public Action intakeOut() { return new IntakeOut(); }
    public Action transferIn() { return new TransferIn(); }
    public Action transferSlow() { return new TransferSlow(); }
    public Action transferOut() { return new TransferOut(); }
    public Action powerDown() { return new PowerDown(); }
    public Action shooterStop() {return new ShooterStop(); }
    public Action intakeStop() {return new IntakeStop(); }
    public Action transferStop() {return new TransferStop(); }



    public class ShooterStop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shooter.setPower(0);
            return false;
        }
    }

    public class IntakeStop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(0);
            return false;
        }
    }

    public class TransferStop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(0);
            return false;
        }
    }

    public class PowerDown implements  Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shooter.setPower(0);
            transfer.setPower(0);
            intake.setPower(0);
            return false;
        }
    }
    public class ShooterPowerUp implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shooter.setPower(1);
            return false;
        }
    }

    public class ShooterPowerDown implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shooter.setPower(0);
            return false;
        }
    }

    public class IntakeIn implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(.85);
            return false;
        }
    }
    public class IntakeOut implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(-.5);
            return false;
        }
    }


    public class TransferIn implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(.8);
            return false;
        }
    }

    public class TransferSlow implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(.45);
            return false;
        }
    }
    public class TransferOut implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(-.5);
            return false;
        }
    }


}
