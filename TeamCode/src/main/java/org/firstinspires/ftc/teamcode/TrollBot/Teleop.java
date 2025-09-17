package org.firstinspires.ftc.teamcode.TrollBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Teleop")
public class Teleop extends OpMode {

    private DcMotor arm;
    private PIDFController controller;
    private FtcDashboard dashboard;

    private double targetTicks = 0;
    private double lastTime = 0;
    private ElapsedTime timer = new ElapsedTime();

    private static final int increment = 6;

    @Override
    public void init() {
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDFController();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void start() {
        timer.reset();
        lastTime = 0;
        targetTicks = arm.getCurrentPosition();
    }

    @Override
    public void loop() {
        // Adjust target with d-pad
        if (gamepad1.dpad_up){
            targetTicks += increment;
        }
        if (gamepad1.dpad_down){
            targetTicks -= increment;
        }
        if (gamepad1.a){
            targetTicks = arm.getCurrentPosition();
        }

        // Change in time!
        double now = timer.seconds();
        double dt = Math.max(1e-3, now - lastTime);
        lastTime = now;

        // Position input and power output
        double pos = arm.getCurrentPosition();
        double power = controller.update(targetTicks, pos, dt); // Yay use PIDContoller

        arm.setPower(power);

        telemetry.addData("Target", targetTicks);
        telemetry.addData("Pos", pos);
        telemetry.addData("Error", targetTicks - pos);
        telemetry.addData("Power", power);

        TelemetryPacket pkt = new TelemetryPacket();
        pkt.put("target", targetTicks);
        pkt.put("pos", pos);
        pkt.put("error", targetTicks - pos);
        pkt.put("power", power);
        pkt.put("kP", PIDFController.kP);
        pkt.put("kI", PIDFController.kI);
        pkt.put("kD", PIDFController.kD);
        pkt.put("kF", PIDFController.kF);
        dashboard.sendTelemetryPacket(pkt);
    }
}