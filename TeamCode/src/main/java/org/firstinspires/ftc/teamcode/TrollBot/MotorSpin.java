package org.firstinspires.ftc.teamcode.TrollBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorSpin extends LinearOpMode {

    private DcMotor motor;
    private DcMotor motora, motorb, motorc;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motora = hardwareMap.get(DcMotorEx.class, "motora");
        motorb = hardwareMap.get(DcMotorEx.class, "motorb");
        motorc = hardwareMap.get(DcMotorEx.class, "motorac");


        waitForStart();
        while(opModeIsActive()){
            if(Math.abs(gamepad1.right_stick_y) > 0.1){
                motor.setPower(gamepad1.right_stick_y);
                motora.setPower(gamepad1.right_stick_y);
                motorb.setPower(gamepad1.right_stick_y);
                motorc.setPower(gamepad1.right_stick_y);
            } else {
                motor.setPower(0);
                motora.setPower(0);
                motorb.setPower(0);
                motorc.setPower(0);
            }
        }


    }
}
