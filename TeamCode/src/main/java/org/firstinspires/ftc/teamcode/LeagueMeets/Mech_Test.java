package org.firstinspires.ftc.teamcode.LeagueMeets;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Mech_Test extends LinearOpMode {

    private DcMotorEx transfer, intake, shooter,shooter2;
    private Servo ramp, gate;
    private IMU imu;
    private double SPEED_MULTIPLIER = 1;
    private double RAMP_EXTEND_LIMIT, RAMP_RETRACT_LIMIT;
    private boolean pastA = false, pastB = false;

    @Override
    public void runOpMode() throws InterruptedException {

        double shooterPower = 0.1;
        double intakePower = 0.7;
        double transferPower = 1;
        double shooterSpeed = 100;

        boolean active_shooter = false;
        Controller_PIDF_Shooter_Close shooter_pid = new Controller_PIDF_Shooter_Close(shooter);

        waitForStart();
        while (opModeIsActive()){
            /// ----------------- Mechanism Controls -----------------
            if (gamepad2.a && !pastA && shooterPower > 0){
                shooterPower -= .1;
            }
            if (gamepad2.b && !pastB && shooterPower < 1){
                shooterPower += .1;
            }

//            if (gamepad2.x & gamepad2.y){
//                shooter.setPower(0.1);
//                shooter2.setPower(0.1);
//            } else if (gamepad2.x){
//                shooter.setPower(1);
//                shooter2.setPower(1);
//            } else if (gamepad2.y){
//                shooter.setPower(0);
//                shooter2.setPower(0);
//            }

            if (gamepad2.x){
                active_shooter = true;
                shooter_pid.setTargetRpm(3250);
            } else  if (gamepad2.y){
                active_shooter = false;
            }


            if (active_shooter){
                double power = shooter_pid.update();
                shooter.setPower(power);
                shooter2.setPower(power);
            } else {
                shooter.setPower(0);
                shooter2.setPower(0);
            }


            if (gamepad2.right_bumper || gamepad1.right_bumper){
                intake.setPower(intakePower);
                //transfer.setPower(transferPower);
            } else if (gamepad2.left_bumper || gamepad1.left_bumper){
                intake.setPower(- intakePower / 2);
                //transfer.setPower(- transferPower / 2);
            } else {
                intake.setPower(0);
                //transfer.setPower(0);
            }

            if (gamepad2.right_trigger > 0.1){
                transfer.setPower(gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0.1){
                transfer.setPower(-transferPower / 2);
            } else if (gamepad1.right_trigger > 0.1){
                transfer.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                transfer.setPower(-transferPower / 2);
            } else {
                transfer.setPower(0);
            }

            pastA = gamepad2.a;
            pastB = gamepad2.b;

        }


    }

    private void initialize(){
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");


        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu.initialize(params);
        imu.resetYaw();


    }
}
