package org.firstinspires.ftc.teamcode.LeagueChampThursday;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.A_Regional.S_CloseShooterPID_Slew;

public class C_Red_Mechanism {
    private DcMotorEx intake, transfer, topShooter, bottomShooter;
    private Servo transferGate, shooterRamp;
    private CRServo turret;
    private S_CloseShooterPID_Slew shooter_pid;
    private Limelight3A limelight;
    private double targetRpm = 3000;

    public static double LL_kP = 0.026;
    public static double LL_kD = 0.001;
    public static double LL_kS_min = 0.03;
    public static double LL_kS_max = 0.12;
    public static double LL_kS_fullAtDeg = 1.0;
    public static double LL_deadbandDeg = 1.25;
    public static double LL_MIN_MOVE_POWER = 0.07;
    public static double LL_MAX_POWER = 0.45;

    // Turret IMU
    //public static double turretOffsetRad = Math.toRadians(139);

    // Limelight
    public static int PIPELINE = 1;
    public static int pollRateHz = 100;

    private double lastErrorRad = 0.0;
    private long lastTimeNs = 0;

    private boolean limelightOn = false;
    private long lastTargetSeenNs = 0;

    private final S_CloseShooterPID shooterPID = new S_CloseShooterPID();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private double int_intake_power = .85,  int_transfer_power = 0.55;
    private double tf_intake_power = 0.5, tf_transfer_power = 0.8;
    private double gateOpen = 0.75, gateClose = 0.63;

    public C_Red_Mechanism(HardwareMap hardwareMap){
        topShooter = hardwareMap.get(DcMotorEx.class, "topShooter");
        bottomShooter = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        topShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        transferGate = hardwareMap.get(Servo.class, "transferGate");

        shooter_pid = new S_CloseShooterPID_Slew();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turret = hardwareMap.get(CRServo.class, "turret");

        shooterRamp = hardwareMap.get(Servo.class, "shooterRamp");
    }






    ///  Simple Actions
    ///

    public Action intake() {return new Intake(); }
    public Action transfer() {return new Transfer(); }
    public Action stop() { return new StopIntnTf(); }
    public Action keepRpm( double tra ) { targetRpm = tra; return new KeepRpm(); }

    public class StopIntnTf implements  Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(0);
            transfer.setPower(0);
            transferGate.setPosition(gateClose);
            return false;
        }
    }
    public class Intake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(int_transfer_power);
            intake.setPower(int_intake_power);

            transferGate.setPosition(gateClose);
            return false;
        }
    }

    public class Transfer implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(tf_transfer_power);
            intake.setPower(tf_intake_power);

            transferGate.setPosition(gateOpen);
            return false;
        }
    }





    ///  Complex Actions
    public Action spinTo(double rpm, double rampPos) {
        targetRpm = rpm;
        shooterRamp.setPosition(rampPos);
        return new SpinTo();
    }

    public Action searchLL () {
        return  new SearchLL();
    }

    public class SpinTo implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shooter_pid.setTargetFlywheelRPM(targetRpm);
            double ticksPerSec = topShooter.getVelocity();
            double measuredFlywheelRPM = S_CloseShooterPID.motorTicksPerSecToFlywheelRPM(ticksPerSec);

            double powerCmd;
            if (!shooter_pid.isAtTarget()) {
                powerCmd = shooter_pid.update(ticksPerSec);
                topShooter.setPower(powerCmd);

                telemetryPacket.put("Target RPM: ", targetRpm);
                telemetryPacket.put("PowerCmd: ", powerCmd);
                telemetryPacket.put("Current: ", topShooter.getVelocity());
                return true;
            }
            return false;
        }
    }

    public class KeepRpm implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shooter_pid.setTargetFlywheelRPM(targetRpm);
            double ticksPerSec = topShooter.getVelocity();
            double measuredFlywheelRPM = S_CloseShooterPID.motorTicksPerSecToFlywheelRPM(ticksPerSec);

            double powerCmd;
            if (!shooter_pid.isAtTarget()) {
                powerCmd = shooter_pid.update(ticksPerSec);
                topShooter.setPower(powerCmd);
                bottomShooter.setPower(powerCmd);
                telemetryPacket.put("Target RPM: ", targetRpm);
                telemetryPacket.put("PowerCmd: ", powerCmd);
                return true;
            }
            return true;
        }
    }

    public class SearchLL implements Action {

        // pid stuff
        public double timeoutSec = 4;

        // values
        private boolean initialized = false;
        private long startNs = 0;
        private long lastSeenNs = 0;
        ElapsedTime timeOfDetection = new ElapsedTime();
        double lockTime = 1;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (!initialized) {
                initialized = true;
                startNs = System.nanoTime();
                lastSeenNs = startNs;

                resetDerivative();
                startLimelight();
            }

            double elapsed = (System.nanoTime() - startNs) / 1e9;
            telemetryPacket.put("LL/elapsed", elapsed);

            if (elapsed >= timeoutSec) {
                turret.setPower(0.0);
                stopLimelight();
                telemetryPacket.put("LL/result", "TIMEOUT");
                return false; // action done
            }

            LLResult result = limelight.getLatestResult();
            boolean hasTarget = (result != null && result.isValid());
            telemetryPacket.put("LL/hasTarget", hasTarget ? 1 : 0);

            if (!hasTarget) {
                turret.setPower(0.0);
                return true;
            }

            lastSeenNs = System.nanoTime();

            double txDeg = result.getTx();
            telemetryPacket.put("LL/txDeg", txDeg);

            double errorRad = Math.toRadians(-txDeg);
            double errorRate = derivative(errorRad);

            double turretPower = limelightCmd(errorRad, errorRate);
            turret.setPower(turretPower);

            telemetryPacket.put("LL/turretPower", turretPower);
            telemetryPacket.put("LL/errorDeg", Math.abs(txDeg));

            if (Math.abs(txDeg) <= LL_deadbandDeg && timeOfDetection.seconds() > lockTime) {
                turret.setPower(0.0);
                stopLimelight();
                telemetryPacket.put("LL/result", "LOCKED");
                return false; // action done
            } else {
                timeOfDetection.reset();
            }

            // keep looping
            return true;
        }
    }

    private double limelightCmd(double errorRad, double errorRateRadPerSec) {
        double absErr = Math.abs(errorRad);
        double deadbandRad = Math.toRadians(LL_deadbandDeg);
        if (absErr <= deadbandRad) return 0.0;

        double fullAtRad = Math.toRadians(LL_kS_fullAtDeg);
        double t = Range.clip(absErr / fullAtRad, 0.0, 1.0);
        double smooth = (3.0 * t * t) - (2.0 * t * t * t);
        double kS = LL_kS_min + (LL_kS_max - LL_kS_min) * smooth;

        double power = (LL_kP * errorRad) + (LL_kD * errorRateRadPerSec) + (kS * Math.signum(errorRad));

        if (Math.abs(power) < LL_MIN_MOVE_POWER) {
            power = LL_MIN_MOVE_POWER * Math.signum(errorRad);
        }

        return Range.clip(power, -LL_MAX_POWER, LL_MAX_POWER);
    }

    private double derivative(double errorRad) {
        long now = System.nanoTime();
        double dt;

        if (lastTimeNs == 0) {
            dt = 0.02;
        } else {
            dt = (now - lastTimeNs) / 1e9;
        }

        lastTimeNs = now;

        if (dt <= 0) dt = 0.02;

        double rate = (errorRad - lastErrorRad) / dt;
        lastErrorRad = errorRad;
        return rate;
    }

    private void resetDerivative() {
        lastErrorRad = 0.0;
        lastTimeNs = 0;
    }

    private static double wrapAngle(double rad) {
        while (rad <= -Math.PI) rad += 2.0 * Math.PI;
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        return rad;
    }

    private void startLimelight() {
        if (limelightOn) return;

        limelight.setPollRateHz(pollRateHz);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        limelightOn = true;
        lastTargetSeenNs = System.nanoTime();
    }

    private void stopLimelight() {
        if (!limelightOn) return;
        limelight.stop();
        limelightOn = false;
    }

}
