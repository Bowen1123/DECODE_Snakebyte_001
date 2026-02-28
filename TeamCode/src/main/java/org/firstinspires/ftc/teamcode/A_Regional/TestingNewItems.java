package org.firstinspires.ftc.teamcode.A_Regional;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * goBILDA RGB Indicator Light (PWM Controlled) tester
 *
 * Configure as a SERVO named "rgb".
 *
 * Controls:
 *  - Dpad Up/Down: step position by STEP
 *  - X: sweep mode toggle (automatically scans 0 -> 1)
 *  - A/B/Y: quick presets (you will change these once you discover colors)
 *  - Left bumper: set to 0.0
 *  - Right bumper: set to 1.0
 */
@TeleOp
public class TestingNewItems extends OpMode {

    private Servo rgb;

    // Adjust if you want finer stepping
    private static final double STEP = 0.01;

    private double pos = 0.0;

    private boolean sweepEnabled = false;
    private boolean lastX = false;

    private boolean lastUp = false, lastDown = false;

    private final ElapsedTime sweepTimer = new ElapsedTime();

    // Sweep speed: seconds between steps
    private static final double SWEEP_DT = 0.06;

    @Override
    public void init() {
        rgb = hardwareMap.get(Servo.class, "led");
        pos = 0.0;
        rgb.setPosition(pos);

        gamepad1.rumble(250);

        telemetry.addLine("Configured as Servo named \"rgb\"");
        telemetry.addLine("Use dpad to step, X to toggle sweep.");
    }

    @Override
    public void loop() {


        if (gamepad1.aWasPressed()) {
            gamepad1.rumble(200,300,400);
        }


//        // Toggle sweep mode with X
//        boolean x = gamepad1.x;
//        if (x && !lastX) {
//            sweepEnabled = !sweepEnabled;
//            sweepTimer.reset();
//        }
//        lastX = x;
//
//        // Presets (you will rewrite these after you learn the mapping)
//        if (gamepad1.a) pos = 0.10;
//        if (gamepad1.b) pos = 0.50;
//        if (gamepad1.y) pos = 0.90;
//
//        if (gamepad1.left_bumper) pos = 0.0;
//        if (gamepad1.right_bumper) pos = 1.0;
//
//        // Manual stepping (edge-detected)
//        boolean up = gamepad1.dpad_up;
//        boolean down = gamepad1.dpad_down;
//
//        if (!sweepEnabled) {
//            if (up && !lastUp) pos += STEP;
//            if (down && !lastDown) pos -= STEP;
//        }
//
//        lastUp = up;
//        lastDown = down;
//
//        // Sweep behavior
//        if (sweepEnabled && sweepTimer.seconds() >= SWEEP_DT) {
//            sweepTimer.reset();
//            pos += STEP;
//            if (pos > 1.0) pos = 0.0;
//        }
//
//        pos = Range.clip(pos, 0.0, 1.0);
//        rgb.setPosition(pos);
//
//        telemetry.addData("Sweep", sweepEnabled ? "ON" : "OFF");
//        telemetry.addData("Position", "%.3f", pos);
//        telemetry.addLine("Tip: Write down which Position gives which color/pattern.");
//        telemetry.update();
    }
}