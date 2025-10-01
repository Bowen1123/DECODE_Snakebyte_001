package org.firstinspires.ftc.teamcode.TrollBot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Teleop_PID_Spinner", group = "TrollBot")
@Config
public class Teleop_PID_Spinner extends OpMode {

    // ---------- Hardware ----------
    public static String MOTOR_NAME = "spinner";
    private DcMotor spinner;

    // ---------- Encoder / RPM calc ----------
    // goBILDA 6000RPM motor: encoder = 28 CPR * 4x = 112 ticks/rev
    public static int TICKS_PER_REV = 112;

    // ---------- Tunables (FTC Dashboard) ----------
    public static double kP = 0.0008;
    public static double kI = 0.00002;
    public static double kD = 0.0000;

    public static double TARGET_RPM = 3000;
    public static double MAX_INTEGRAL = 0.5;
    public static double ERROR_DEADBAND_RPM = 5.0;

    public static double MIN_OUTPUT = 0.0; // cant spin in revers!
    public static double MAX_OUTPUT = 1.0;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastTime = 0.0;
    private int lastTicks = 0;

    private boolean pidEnabled = true;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastDpadUp = false, lastDpadDown = false, lastDpadLeft = false, lastDpadRight = false;

    private PIDSpinnerController controller;

    @Override
    public void init() {
        spinner = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDSpinnerController(kP, kI, kD);
        controller.MIN_OUTPUT = MIN_OUTPUT;
        controller.MAX_OUTPUT = MAX_OUTPUT;
        controller.MAX_INTEGRAL = MAX_INTEGRAL;
        controller.ERROR_DEADBAND_RPM = ERROR_DEADBAND_RPM;

        lastTicks = spinner.getCurrentPosition();
        loopTimer.reset();
        lastTime = loopTimer.seconds();

        telemetry.addLine("Teleop_PID_Spinner init complete.");
        telemetry.update();

        FtcDashboard.getInstance(); // enable dashboard
    }

    @Override
    public void start() {
        loopTimer.reset();
        lastTime = 0.0;
        lastTicks = spinner.getCurrentPosition();
        controller.reset();
    }

    @Override
    public void loop() {
        // Handle button presses
        if (gamepad1.a && !lastA) {
            pidEnabled = !pidEnabled;
        }
        if (gamepad1.b && !lastB) {
            controller.reset();
        }

        if (gamepad1.dpad_up && !lastDpadUp) {
            TARGET_RPM += 100;
        }
        if (gamepad1.dpad_down && !lastDpadDown){
            TARGET_RPM = Math.max(0, TARGET_RPM - 100);
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            TARGET_RPM += 25;
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            TARGET_RPM = Math.max(0, TARGET_RPM - 25);
        }

        lastA = gamepad1.a; lastB = gamepad1.b;
        lastDpadUp = gamepad1.dpad_up; lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left; lastDpadRight = gamepad1.dpad_right;

        // Compute dt and RPM
        double now = loopTimer.seconds();
        double dt = now - lastTime;
        if (dt <= 0) dt = 1e-3;

        int ticks = spinner.getCurrentPosition();
        int deltaTicks = ticks - lastTicks;
        double ticksPerSec = deltaTicks / dt;
        double currentRpm = (ticksPerSec / TICKS_PER_REV) * 60.0;

        lastTime = now;
        lastTicks = ticks;

        // Update gains live
        controller.setGains(kP, kI, kD);
        controller.MIN_OUTPUT = MIN_OUTPUT;
        controller.MAX_OUTPUT = MAX_OUTPUT;
        controller.MAX_INTEGRAL = MAX_INTEGRAL;
        controller.ERROR_DEADBAND_RPM = ERROR_DEADBAND_RPM;

        double power;
        if (pidEnabled) {
            power = controller.update(TARGET_RPM, currentRpm, dt);
        } else {
            power = 0.0;
        }
        spinner.setPower(power);

        // Telemetry
        telemetry.addLine("== PID Spinner ==");
        telemetry.addData("Enabled", pidEnabled);
        telemetry.addData("Target RPM", "%.1f", TARGET_RPM);
        telemetry.addData("Current RPM", "%.1f", currentRpm);
        telemetry.addData("kP/kI/kD", "%.6f / %.6f / %.6f", kP, kI, kD);
        telemetry.addData("Output (power)", "%.3f", power);
        telemetry.addData("dt (s)", "%.4f", dt);
        telemetry.addLine("Controls: A=toggle PID, B=reset, Dpad +/- RPM");
        telemetry.update();
    }

    @Override
    public void stop() {
        spinner.setPower(0.0);
    }
}