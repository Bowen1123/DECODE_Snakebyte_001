package org.firstinspires.ftc.teamcode.A_Regional;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(group = "Turret")
public class D_TurretTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "limelight");
        CRServo turretServo = hardwareMap.get(CRServo.class, "turret");

        D_Turret turret = new D_Turret(ll);
        turret.init(turretServo);
        turret.start();
        turret.setTrackingEnabled(false);

        FtcDashboard dash = FtcDashboard.getInstance();

        waitForStart();

        boolean lastA = false;
        boolean lastB = false;

        while (opModeIsActive()) {

            // Always update LL first
            turret.updateLimelight();

            // Toggle tracking
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            if (a && !lastA) turret.setTrackingEnabled(!turret.isTrackingEnabled());
            if (b && !lastB) turret.setTrackingEnabled(false);

            lastA = a;
            lastB = b;

            boolean aimed = turret.loop();

            // Dashboard packet (nice for tuning)
            TelemetryPacket p = new TelemetryPacket();
            p.put("trackingEnabled", turret.isTrackingEnabled());
            p.put("hasTarget", turret.hasTarget());
            p.put("aimed(deadband)", aimed);

            p.put("tx_raw_deg", turret.getTxDeg());
            p.put("tx_filt_deg", turret.getTxFilteredDeg());
            p.put("ty_deg", turret.getTyDeg());

            p.put("out", turret.getLastOut());
            p.put("distance_in", turret.getGroundDistanceInches());

            dash.sendTelemetryPacket(p);

            // Driver station telemetry (quick glance)
            telemetry.addData("tracking", turret.isTrackingEnabled());
            telemetry.addData("hasTarget", turret.hasTarget());
            telemetry.addData("aimed", aimed);
            telemetry.addData("tx raw", turret.getTxDeg());
            telemetry.addData("tx filt", turret.getTxFilteredDeg());
            telemetry.addData("out", turret.getLastOut());
            telemetry.addData("dist(in)", turret.getGroundDistanceInches());
            telemetry.update();
        }

        turret.setTrackingEnabled(false);
        turret.stop();
    }
}