package org.firstinspires.ftc.teamcode.TrollBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
public class ShooterTestH extends LinearOpMode {
    public DcMotorEx shooter, shooter2;
    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(DcMotorEx.class,  "shooter");
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.dpad_up){
                shooter.setPower(-1);
            }
            if(gamepad1.dpad_down){
                shooter.setPower(1);
            }
            if(gamepad1.dpad_left){
                shooter.setPower(-.5);
            }
            if(gamepad1.dpad_right){
                shooter.setPower(.5);
            }
            if(gamepad1.b){
                shooter.setPower(0);
            }
        }
    }
}
