package org.firstinspires.ftc.teamcode.TrollBot;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class Test_PushCount extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, transfer, shooter, intake;
    private Servo ramp;
    private IMU imu;
    private double SPEED_MULTIPLIER = 1;
    private double RAMP_EXTEND_LIMIT, RAMP_RETRACT_LIMIT;

    private boolean pastA = false, pastB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        initialize();

        ElapsedTime timer = new ElapsedTime();
        Pose2d initialPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        double shooterPower = 0.1;
        double intakePower = 0.85;
        double transferPower = .85;
        double shooterSpeed = 100;
        double lfE = 0;
        while (opModeIsActive()){
            lfE = leftFront.getCurrentPosition();
            telemetry.addData("Test", lfE);
            telemetry.update();
        }
    }
    private double getHeadingDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    private void moveRobot(double x, double y, double yaw) {
        double fl =  y - x - yaw;
        double fr =  y + x + yaw;
        double bl =  y + x - yaw;
        double br =  y - x + yaw;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        leftFront.setPower(fl * SPEED_MULTIPLIER);
        rightFront.setPower(fr * SPEED_MULTIPLIER);
        leftBack.setPower(bl * SPEED_MULTIPLIER);
        rightBack.setPower(br * SPEED_MULTIPLIER);
    }
    private void initialize(){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");



        //transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        // shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
