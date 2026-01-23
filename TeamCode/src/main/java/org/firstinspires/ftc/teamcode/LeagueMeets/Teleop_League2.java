package org.firstinspires.ftc.teamcode.LeagueMeets;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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
public class Teleop_League2 extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, transfer, shooter, intake, shooter2;
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

        boolean active_shooter = false;
        Controller_PIDF_Shooter_Close shooter_pid = new Controller_PIDF_Shooter_Close(shooter);

        double shooterPower = 0.1;
        double intakePower = 0.85;
        double transferPower = .85;
        double shooterSpeed = 100;

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


            /// ----------------- Field-Centric Drive -----------------
            /*double driveForward = -gamepad1.left_stick_y;  // forward/back
            double strafe =  -gamepad1.left_stick_x;  // left/right
            double driveTurn; // ccw/cw
            if (Math.abs(gamepad1.left_stick_x) < Math.abs(gamepad2.left_stick_x) && Math.abs(gamepad2.left_stick_x) > 0.1){
                driveTurn = -gamepad2.right_stick_x;
            } else {
                driveTurn = -gamepad1.right_stick_x;
            }



            double xPower, yPower;
            double headingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double fieldY = driveForward;
            double fieldX = strafe;
            double robotY =  fieldY * Math.cos(headingRad) + fieldX * Math.sin(headingRad);
            double robotX = -fieldY * Math.sin(headingRad) + fieldX * Math.cos(headingRad);

            yPower = robotY;
            xPower = robotX;
            moveRobot(xPower, yPower, driveTurn);*/
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * .75,
                            -gamepad1.left_stick_x * .75
                    ),
                    -gamepad1.right_stick_x * .75
            ));



            /// ----------------- Update Previous Gamepads -----------------
            pastA = gamepad2.a;
            pastB = gamepad2.b;

            /// ----------------- Update Telemetry -----------------
            telemetry.addLine("Shooter Power: " + shooterPower);
            telemetry.addLine("Intake Power: " + intakePower);
            telemetry.addLine("Transfer Power: " + transferPower);
            telemetry.addLine("\n\nHeading: " + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
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
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        // shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

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
