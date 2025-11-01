package org.firstinspires.ftc.teamcode.TrollBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Prototype_Spinner extends LinearOpMode {

    private DcMotor spinner;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        spinner  = hardwareMap.get(DcMotor.class, "spinner");
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()){

            if (Math.abs(gamepad1.left_stick_y) >0.2){
                spinner.setPower(Math.abs(gamepad1.left_stick_y));
            } else {
                spinner.setPower(0);
            }

        }

    }
}
