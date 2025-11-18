package org.firstinspires.ftc.teamcode.TrollBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Teleop_PIDF_Arm")
public class Test_PIDF_Arm extends OpMode {

    private DcMotor arm;
    private PIDF_ArmController controller;
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

        controller = new PIDF_ArmController();
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
        pkt.put("kP", PIDF_ArmController.kP);
        pkt.put("kI", PIDF_ArmController.kI);
        pkt.put("kD", PIDF_ArmController.kD);
        pkt.put("kF", PIDF_ArmController.kF);
        dashboard.sendTelemetryPacket(pkt);
    }
}