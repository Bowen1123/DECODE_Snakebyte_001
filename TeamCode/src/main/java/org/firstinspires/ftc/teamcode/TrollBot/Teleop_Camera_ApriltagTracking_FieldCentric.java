package org.firstinspires.ftc.teamcode.TrollBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp(name = "ApriltagTracking_FieldCentric")
public class Teleop_Camera_ApriltagTracking_FieldCentric extends OpMode {

    public static double SPEED_MULTIPLIER = 0.9;

    public static double TARGET_RANGE = 96.0;

    public static double BEARING_TOLERANCE = 2.0;   // must be within this bearing error to move
    public static double RANGE_TOLERANCE = 1.0;     // stop translating when within this distance
    public static double FORWARD_K     = 0.015;     // power per inch of range error
    public static double MAX_FWD_POWER = 0.7;       // cap on forward/back power while driving to range

    public static double MAX_TURN_POWER = 0.6;      // cap for heading PID output

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private IMU imu;

    private static final boolean USE_WEBCAM = true;
    public static int DESIRED_TAG_ID = 20;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag;

    private final PID_HeadingController headingPid = new PID_HeadingController(0.020, 0.000, 0.001);
    private double lastPidTurn = 0.0;

    @Override
    public void init() {
        // Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft   = hardwareMap.get(DcMotor.class, "leftBack");
        backRight  = hardwareMap.get(DcMotor.class, "rightBack");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(params);
        imu.resetYaw();

        // Vision
        initAprilTag();

        telemetry.addLine("AprilTag Tracking (Field-Centric + DriveTo)");
        telemetry.addData("Desired Tag ID", DESIRED_TAG_ID);
        telemetry.addLine("RB: heading lock to tag | LB: drive-to (aim then go straight)");
        telemetry.update();
    }

    @Override
    public void start() {
        headingPid.reset();
        lastPidTurn = 0.0;
    }

    @Override
    public void loop() {
        // ---------------- Driver inputs ----------------
        double driveY = -gamepad1.left_stick_y;  // forward/back
        double driveX =  gamepad1.left_stick_x;  // left/right
        double driveTurn = gamepad1.right_stick_x; // ccw/cw

        // ---------------- Vision ----------------
        boolean targetFound = false;
        desiredTag = null;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.metadata != null && d.id == DESIRED_TAG_ID) {
                desiredTag = d;
                targetFound = true;
                break;
            }
        }

        // ---------------- Modes ----------------
        boolean lockRequested    = gamepad1.right_bumper && targetFound;
        boolean driveToRequested = gamepad1.left_bumper  && targetFound;

        // --- Heading control (used by both lock and drive-to) ---
        double turnPower;
        if ((lockRequested || driveToRequested) && targetFound) {
            double bearingDeg = desiredTag.ftcPose.bearing;
            double pidOut = headingPid.update(0.0, bearingDeg); // 0° = directly at tag
            pidOut = Range.clip(pidOut, -MAX_TURN_POWER, MAX_TURN_POWER);
            lastPidTurn = pidOut;
            turnPower = pidOut;
        } else {
            turnPower = driveTurn;
        }

        double xPower, yPower;

        if (driveToRequested && targetFound) {
            double bearingDeg = desiredTag.ftcPose.bearing;
            boolean aimed = Math.abs(bearingDeg) <= BEARING_TOLERANCE;

            // Range control: go straight forward/back toward target distance
            double rangeErr = TARGET_RANGE - desiredTag.ftcPose.range;
            if (Math.abs(rangeErr) <= RANGE_TOLERANCE) {
                yPower = 0.0;
            } else if (aimed) {
                double fwdPower = Range.clip(FORWARD_K * rangeErr, -MAX_FWD_POWER, MAX_FWD_POWER);
                yPower = fwdPower;
            } else {
                yPower = 0.0;
            }

            // no strafe in drive-to
            xPower = 0.0;

        } else {
            // Field-centric transform
            double headingRad = Math.toRadians(getHeadingDeg());

            double fieldY = driveY;
            double fieldX = driveX;

            double robotY =  fieldY * Math.cos(headingRad) + fieldX * Math.sin(headingRad);
            double robotX = -fieldY * Math.sin(headingRad) + fieldX * Math.cos(headingRad);

            yPower = robotY;
            xPower = robotX;
        }

        // ---------------- Drive ----------------
        moveRobot(xPower, yPower, turnPower);

        // ---------------- Telemetry ----------------
        telemetry.addData("Heading (deg)", "%.1f", getHeadingDeg());
        telemetry.addData("Desired ID", DESIRED_TAG_ID);
        telemetry.addData("Tag Visible", targetFound);
        telemetry.addData("Lock", lockRequested ? "ON (RB)" : "OFF");
        telemetry.addData("DriveTo", driveToRequested ? "ON (LB)" : "OFF");
        if (targetFound) {
            double bearingDeg = desiredTag.ftcPose.bearing;
            double rangeErr   = TARGET_RANGE - desiredTag.ftcPose.range;
            telemetry.addData("Tag", desiredTag.metadata.name);
            telemetry.addData("Range (in)", "%.1f", desiredTag.ftcPose.range);
            telemetry.addData("Bearing (deg)", "%.1f", bearingDeg);
            telemetry.addData("Aim OK?", Math.abs(bearingDeg) <= BEARING_TOLERANCE);
            telemetry.addData("Range Err (in)", "%.2f", rangeErr);
            telemetry.addData("PID Turn", "%.3f", lastPidTurn);
        } else {
            telemetry.addLine("Rotate/drive to find the desired tag.");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    // ---------------- Helpers ----------------
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagID(true)
                .build();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
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

    /* Mecanum drive using robot-centric y (forward), x (strafe), yaw (ccw is positive) */
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

        frontLeft.setPower(fl * SPEED_MULTIPLIER);
        frontRight.setPower(fr * SPEED_MULTIPLIER);
        backLeft.setPower(bl * SPEED_MULTIPLIER);
        backRight.setPower(br * SPEED_MULTIPLIER);
    }

    private double getHeadingDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}