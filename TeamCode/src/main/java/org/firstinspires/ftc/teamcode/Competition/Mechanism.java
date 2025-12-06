package org.firstinspires.ftc.teamcode.Competition;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mechanism {

    private DcMotorEx shooter, intake, transfer;

    private Controller_PIDF_Shooter_Close shooter_pid;
    private boolean active_shooter = false;

    public Mechanism(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        shooter_pid = new Controller_PIDF_Shooter_Close(shooter);
    }

    private double targetRPM = 0;

    public Action shooterSpinTo(double targetRPM){this.targetRPM = targetRPM; active_shooter = true; return new ShooterSpinTo();}
    public Action shooterPowerUp() { return new ShooterPowerUp(); }
    public Action shooterPowerDown() { return  new ShooterPowerDown(); }
    public Action intakeIn() { return new IntakeIn(); }
    public Action intakeSlow() {return new IntakeSlow(); }
    public Action intakeOut() { return new IntakeOut(); }
    public Action transferIn() { return new TransferIn(); }
    public Action transferInFinal() { return new TransferInFinal(); }

    public Action transferSlow() { return new TransferSlow(); }
    public Action transferSlowest() { return new TransferSlowest(); }
    public Action transferOut() { return new TransferOut(); }
    public Action powerDown() { return new PowerDown(); }
    public Action powerDownIntake() {return new PowerDownIntake(); }
    public Action intakeStop() {return new IntakeStop(); }
    public Action transferStop() {return new TransferStop(); }


    public class ShooterSpinTo implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (active_shooter){
                shooter_pid.setTargetRpm(targetRPM);
                shooter.setPower(shooter_pid.update());
                return true; /// I think true means it will loop, false it will end
            }
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

    public class PowerDownIntake implements  Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
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

    public class IntakeSlow implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(.45);
            return false;
        }
    }
    public class IntakeOut implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(-.6);
            return false;
        }
    }


    public class TransferIn implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(.9);
            return false;
        }
    }

    public class TransferInFinal implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(1);
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
    public class TransferSlowest implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(.15);
            return false;
        }
    }
    public class TransferOut implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(-.55);
            return false;
        }
    }


}
