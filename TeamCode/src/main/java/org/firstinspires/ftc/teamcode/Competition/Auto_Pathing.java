package org.firstinspires.ftc.teamcode.Competition;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class Auto_Pathing extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, transfer, shooter, intake;
    private Servo ramp;
    private IMU imu;
    private double SPEED_MULTIPLIER = 1;
    private double RAMP_EXTEND_LIMIT, RAMP_RETRACT_LIMIT;

    private boolean pastA = false, pastB = false;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder defaultPath = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(35, 35), Math.toRadians(270));


        waitForStart();
        while(opModeIsActive()){

            if (gamepad1.a && !pastA){
                Actions.runBlocking(defaultPath.build());
            }

            double driveForward = -gamepad1.left_stick_y;  // forward/back
            double strafe =  -gamepad1.left_stick_x;  // left/right
            double driveTurn = -gamepad1.right_stick_x; // ccw/cw

            double xPower, yPower;
            double headingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double fieldY = driveForward;
            double fieldX = strafe;
            double robotY =  fieldY * Math.cos(headingRad) + fieldX * Math.sin(headingRad);
            double robotX = -fieldY * Math.sin(headingRad) + fieldX * Math.cos(headingRad);

            yPower = robotY;
            xPower = robotX;
            moveRobot(xPower, yPower, driveTurn);
            pastA = gamepad1.a;
            pastB = gamepad1.b;

        }
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

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        //shooter.setDirection(DcMotorSimple.Direction.REVERSE);
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
