package org.firstinspires.ftc.teamcode.LeagueChamp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class A_PowerDistributionTeleop extends LinearOpMode {

    ///  Hardware
    private DcMotorEx intake, transfer, topShooter, bottomShooter;
    private CRServo turret;
    private Servo transferGate;
    private IMU turretImu;
    private Limelight3A limelight;

    ///  Mode Settings
    private enum MODE { INTAKE, SHOOT, ENDGAME};
    private MODE mode = MODE.INTAKE;

    // Intake mode limit power to shooters and disables PID controller, limit power to transfer, close servo gate
    // Shoot mode limit power to drivetrain for minimal movement, limit power to intake, open servo gate

    private final double INT_shooter_power = 0.4, INT_transfer_power = 0.4, INT_intake_power = .85, INT_dt_max_power = 0.8;
    private final double SH_shooter_power = 1, SH_transfer_power = 1, SH_intake_power = .5, SH_dt_max_power = 0.4;
    private double shooter_power = 0, transfer_power = 0, intake_power = 0, dt_max_power = 0;


    // Boolean values
    private boolean lastA = false, lastB = false, lastX = false, lastY = false;
    private boolean a1 = false, b1 = false, x1 = false, y1 = false;
    private boolean active_shooter = false;

    @Override
    public void runOpMode() throws InterruptedException {


        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        transferGate = hardwareMap.get(Servo.class, "transferGate");


        turret = hardwareMap.get(CRServo.class, "turret");
        turretImu = hardwareMap.get(IMU.class, "turretImu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        topShooter = hardwareMap.get(DcMotorEx.class, "topShooter");
        bottomShooter = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");


        topShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        topShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Adjust if motors fight each other
        topShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);



        S_CloseShooterPID shooterPID = new S_CloseShooterPID();
        double targetRPM = 3000;     // flywheel RPM target
        double measuredFlywheelRPM = 0;

        double gateOpenPos = 0.55;
        double gateClosePos = 0.25;

        waitForStart();
        while (opModeIsActive()){
            switch (mode) {
                case INTAKE:
                    intake_power = INT_intake_power;
                    shooter_power = INT_shooter_power;
                    transfer_power = INT_transfer_power;
                    dt_max_power = INT_dt_max_power;

                    transferGate.setPosition(gateClosePos);

                    break;
                case SHOOT:
                    intake_power = SH_intake_power;
                    shooter_power = SH_shooter_power;
                    transfer_power = SH_transfer_power;
                    dt_max_power = SH_dt_max_power;

                    active_shooter = true;

                    transferGate.setPosition(gateOpenPos);

                    break;
                case ENDGAME:
                    intake_power = 0.85;
                    shooter_power = 1;
                    transfer_power = 1;
                    dt_max_power = 1;
            }


            ///  Drive ///
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            Range.clip(-gamepad1.left_stick_y, -dt_max_power, dt_max_power),
                            Range.clip(-gamepad1.left_stick_x, -dt_max_power, dt_max_power)
                    ),
                    Range.clip(-gamepad1.right_stick_x, -dt_max_power, dt_max_power)
            ));
            ///  ---------- haha ------------ ///


            ///  Control ///  give to driver two later
            if (gamepad1.right_bumper){
                intake.setPower(intake_power);
                transfer.setPower(transfer_power);
            } else if (gamepad1.left_bumper){
                intake.setPower(-intake_power);
                transfer.setPower(-transfer_power);
            } else {
                intake.setPower(0);
                transfer.setPower(0);
            }


            a1 = gamepad1.a;
            b1 = gamepad1.b;
            x1 = gamepad1.x;
            y1 = gamepad1.y;

            if (a1 && !lastA) {
                mode = MODE.INTAKE;
                active_shooter = false;
            } else if (b1 && !lastB) {
                mode = MODE.SHOOT;
            } else if (y1 && !lastY) {
                mode = MODE.ENDGAME;
                active_shooter = false;
            }

            lastA = a1;
            lastB = b1;
            lastX = x1;
            lastY = y1;



            ///   Shoot   ///
            if (active_shooter && mode.equals(MODE.SHOOT)){
                targetRPM = S_Adaptive_Equations.getFlywheelRPM(64); // need to put in distance someway

                shooterPID.setTargetFlywheelRPM(targetRPM);

                double ticksPerSec = topShooter.getVelocity();
                // measuredFlywheelRPM = S_CloseShooterPID.motorTicksPerSecToFlywheelRPM(ticksPerSec);

                double powerCmd;
                if (targetRPM > 0) {
                    powerCmd = shooterPID.update(ticksPerSec);
                } else {
                    powerCmd = 0.0;
                }

                topShooter.setPower(powerCmd);
                bottomShooter.setPower(powerCmd);
            } else if (mode.equals(MODE.INTAKE) || mode.equals(MODE.ENDGAME)){

            } else {
                topShooter.setPower(0);
                bottomShooter.setPower(0);
            }

        }
    }
}
