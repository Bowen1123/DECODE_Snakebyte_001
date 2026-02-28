package org.firstinspires.ftc.teamcode.A_Regional;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Minimal tuner TeleOp for D_TurretBasicPID
 * - A toggles tracking
 * - B disables tracking
 * - Tune KP/KI/KD/DEADBAND/MAX_POWER live in FTC Dashboard
 */
@TeleOp(group = "Turret")
public class D_BasicTurretTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "limelight");
        CRServo turretServo = hardwareMap.get(CRServo.class, "turret");

        D_BasicTurret turret = new D_BasicTurret(ll);
        turret.init(turretServo);
        turret.start();
        turret.setTrackingEnabled(false);

        FtcDashboard dash = FtcDashboard.getInstance();

        waitForStart();

        boolean lastA = false, lastB = false;

        while (opModeIsActive()) {
            turret.updateLimelight();

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            if (a && !lastA) turret.setTrackingEnabled(!turret.isTrackingEnabled());
            if (b && !lastB) turret.setTrackingEnabled(false);

            lastA = a;
            lastB = b;

            boolean aimed = turret.loop();

            TelemetryPacket p = new TelemetryPacket();
            p.put("trackingEnabled", turret.isTrackingEnabled());
            p.put("hasTarget", turret.hasTarget());
            p.put("aimed", aimed);
            p.put("tx_deg", turret.getTxDeg());
            p.put("ty_deg", turret.getTyDeg());
            dash.sendTelemetryPacket(p);

            telemetry.addData("tracking", turret.isTrackingEnabled());
            telemetry.addData("hasTarget", turret.hasTarget());
            telemetry.addData("aimed", aimed);
            telemetry.addData("tx", turret.getTxDeg());
            telemetry.update();
        }

        turret.setTrackingEnabled(false);
        turret.stop();
    }
}