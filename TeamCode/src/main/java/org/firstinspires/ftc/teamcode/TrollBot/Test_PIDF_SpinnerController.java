package org.firstinspires.ftc.teamcode.TrollBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class Test_PIDF_SpinnerController extends LinearOpMode {

    public static class Params{
        public double kP = 0;
        public double kI = 0;
        public double kD = 0;
        public double kF = 0;
    }

    private static final double TICKS_PER_REV = 28.0; // 6000 RPM

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx spinner = hardwareMap.get(DcMotorEx.class, "spinner");

        spinner.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        PIDF_SpinnerController rpmController = new PIDF_SpinnerController(
                0.02,  // kP
                0.0,    // kI
                0.0,    // kD
                0.0005  // kF (starting guess)
        );
        rpmController.setIntegralLimit(500); // prevent crazy windup

        // Example: start with 1000 RPM target
        double targetRpm = 1000.0;
        rpmController.setTargetRpm(targetRpm);

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        runtime.reset();
        rpmController.reset();

        boolean prevY = false, prevA = false;

        while (opModeIsActive()) {
            double timeS = runtime.seconds();

            // Read current motor speed in ticks/sec and convert to RPM
            double ticksPerSec = spinner.getVelocity(); // by default, ticks/second
            double currentRpm = (ticksPerSec / TICKS_PER_REV) * 60.0;

            // Update controller
            double power = rpmController.update(currentRpm, timeS);

            // Clip power to valid range
            power = Math.max(-1.0, Math.min(1.0, power));
            spinner.setPower(power);

            // spinner.setPower(.2);

            if (gamepad1.y && !prevY) {
                targetRpm += 50;
                rpmController.setTargetRpm(targetRpm);
            } else if (gamepad1.a && !prevA) {
                targetRpm -= 50;
                if (targetRpm < 0) targetRpm = 0;
                rpmController.setTargetRpm(targetRpm);
            }

            prevY = gamepad1.y;
            prevA = gamepad1.a;

            telemetry.addData("Target RPM", targetRpm);
            telemetry.addData("Current RPM", currentRpm);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}