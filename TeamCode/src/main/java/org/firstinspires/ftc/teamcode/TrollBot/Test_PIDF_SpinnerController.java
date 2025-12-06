package org.firstinspires.ftc.teamcode.TrollBot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Competition.Controller_PIDF_Shooter_Close;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Test_PIDF_SpinnerController")
@Config
public class Test_PIDF_SpinnerController extends LinearOpMode {

    public static double INITIAL_TARGET_RPM = 4000;

    private Controller_PIDF_Shooter_Close shooterController;

    private boolean pastA = false;
    private boolean pastY = false;
    private boolean usingCamera = false, pastBumper = false;


    /// With Camera Code ///
    public static int DESIRED_TAG_ID = 21; // <-- set the only tag to lock onto
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag;


    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware
        DcMotorEx spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);


        shooterController = new Controller_PIDF_Shooter_Close(spinner);
        shooterController.setTargetRpm(INITIAL_TARGET_RPM);

        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        telemetry.addLine("Shooter PIDF Test");
        telemetry.addLine("Use A/Y to adjust target RPM.");
        telemetry.update();

        initAprilTag();

        boolean targetFound = false;

        waitForStart();
        while (opModeIsActive()) {
            // ----- Input: edge-detect A/Y for one-step changes -----
            boolean a = gamepad1.a;
            boolean y = gamepad1.y;
            boolean bumpers = gamepad1.right_bumper || gamepad1.left_bumper;

            // A pressed: increase target RPM
            if (y && !pastY) {
                shooterController.setTargetRpm(
                        shooterController.getTargetRpm() + Controller_PIDF_Shooter_Close.RPM_STEP
                );
            }

            // Y pressed: decrease target RPM
            if (a && !pastA) {
                shooterController.setTargetRpm(
                        shooterController.getTargetRpm() - Controller_PIDF_Shooter_Close.RPM_STEP
                );
            }

            pastA = a;
            pastY = y;
            pastBumper = bumpers;

            // ----- Update controller -----
            double currentPower = shooterController.update();
            spinner.setPower(currentPower);






            // ----- Telemetry -----
            telemetry.addData("Target RPM", shooterController.getTargetRpm());
            telemetry.addData("Current RPM", shooterController.getCurrentRpm());
            telemetry.addData("Raw encoder pos", shooterController.getMotor().getCurrentPosition());

            telemetry.addData("Power", currentPower);
            telemetry.addData("At Target (±%.0f RPM)", Controller_PIDF_Shooter_Close.RPM_TOLERANCE);
            telemetry.addData("At Target?", shooterController.isAtTarget());
            telemetry.update();
        }
    }


    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagID(true)
                // You can tweak decimation for range vs FPS
                .build();
        aprilTag.setDecimation(2);

        if (usingCamera) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
}