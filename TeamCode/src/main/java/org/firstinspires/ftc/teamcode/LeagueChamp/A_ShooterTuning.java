package org.firstinspires.ftc.teamcode.LeagueChamp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class A_ShooterTuning extends OpMode {

    private DcMotorEx topShooter;
    private DcMotorEx bottomShooter;
    private DcMotorEx intake, transfer;

    private Servo transferGate;

    private final S_CloseShooterPID shooterPID = new S_CloseShooterPID();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    private double targetRPM = 3000;     // flywheel RPM target
    private boolean shooterEnabled = false;

    // edge detect
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;

    @Override
    public void init() {
        topShooter = hardwareMap.get(DcMotorEx.class, "topShooter");
        bottomShooter = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        transferGate = hardwareMap.get(Servo.class, "transferGate" );


        topShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        topShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Adjust if motors fight each other
        topShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterPID.reset();

        telemetry.addLine("CloseShooterPIDTeleOp ready");
        telemetry.addLine("X toggle shooter | A +100 RPM | B -100 RPM");
        telemetry.addLine("Tune CFG_* in FTC Dashboard under CloseShooterPID");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Live-tune gains/limits from dashboard
        shooterPID.syncFromDashboard();

        // Edge-detect buttons
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;

        if (gamepad1.right_bumper){
            intake.setPower(.9);
        } else if (gamepad1.left_bumper) {
            intake.setPower(-.7);
        } else {
            intake.setPower(0);
        }

        if (gamepad1.right_trigger > 0.2){
            transfer.setPower(1);
        } else if (gamepad1.left_trigger > 0.2){
            transfer.setPower(-.7);
        } else {
            transfer.setPower(0);
        }


        if (a && !lastA) targetRPM += 100.0;
        if (b && !lastB) targetRPM -= 100.0;
        if (x && !lastX) {
            shooterEnabled = !shooterEnabled;
            shooterPID.reset(); // clean start/stop
        }

        lastA = a;
        lastB = b;
        lastX = x;

        // Clamp target
        if (targetRPM < 0) targetRPM = 0;
        if (targetRPM > S_CloseShooterPID.CFG_maxTargetFlywheelRPM) {
            targetRPM = S_CloseShooterPID.CFG_maxTargetFlywheelRPM;
        }

        shooterPID.setTargetFlywheelRPM(targetRPM);

        // Read encoder velocity (ticks/sec) from topShooter
        double ticksPerSec = topShooter.getVelocity();
        double measuredFlywheelRPM = S_CloseShooterPID.motorTicksPerSecToFlywheelRPM(ticksPerSec);

        double powerCmd;
        if (shooterEnabled && targetRPM > 0) {
            powerCmd = shooterPID.update(ticksPerSec);
            transferGate.setPosition(.75);

        } else {
            powerCmd = 0.0;
            transferGate.setPosition(.6);
        }

        topShooter.setPower(powerCmd);
        bottomShooter.setPower(powerCmd);

        telemetry.addData("Shooter", shooterEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Target Flywheel RPM", "%.0f", targetRPM);
        telemetry.addData("Measured Flywheel RPM", "%.0f", measuredFlywheelRPM);
        telemetry.addData("Power Cmd", "%.3f", powerCmd);

        telemetry.addLine("--- Dashboard (CloseShooterPID) ---");
        telemetry.addData("kP", shooterPID.getkP());
        telemetry.addData("kI", shooterPID.getkI());
        telemetry.addData("kD", shooterPID.getkD());
        telemetry.addData("kF", shooterPID.getkF());
        telemetry.addData("min/max", "%.2f / %.2f", shooterPID.getMinPower(), shooterPID.getMaxPower());

        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();

        // Numbers you want graphed:
        packet.put("targetRPM", targetRPM);
        packet.put("currentRPM", measuredFlywheelRPM);
        packet.put("powerCmd", powerCmd);

        // Optional extras:
        packet.put("errorRPM", targetRPM - measuredFlywheelRPM);
        packet.put("enabled", shooterEnabled ? 1 : 0); // graphable boolean

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        topShooter.setPower(0);
        bottomShooter.setPower(0);
    }
}