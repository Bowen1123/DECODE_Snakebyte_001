package org.firstinspires.ftc.teamcode.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutoShoot extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, transfer, shooter, intake;
    private Servo ramp;
    private IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        double lfE= leftBack.getCurrentPosition();;
        while (opModeIsActive()) {
            telemetry.addData("Test", lfE);
            lfE = leftFront.getCurrentPosition();
            telemetry.update();
        }
        /*
        while(lfE < 300){
            lfE = leftFront.getCurrentPosition();
            leftFront.setPower(-1);
            leftBack.setPower(-1);
            rightFront.setPower(-1);
            rightBack.setPower(-1);
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        shooter.setPower(1);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfE= leftFront.getCurrentPosition();
        while(lfE < 300){
            lfE = leftFront.getCurrentPosition();
            leftFront.setPower(0.3);
            leftBack.setPower(0.3);
            rightFront.setPower(-.3);
            rightBack.setPower(-.3);
        }
        transfer.setPower(0.5);
        wait(50);
        */


    }
}
