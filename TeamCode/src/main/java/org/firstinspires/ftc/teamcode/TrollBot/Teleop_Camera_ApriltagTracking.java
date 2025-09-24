package org.firstinspires.ftc.teamcode.TrollBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "ApriltagTracking")
public class Teleop_Camera_ApriltagTracking extends OpMode {

    // ----------- Drivetrain -----------
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ----------- Vision -----------
    private static final boolean USE_WEBCAM = true;
    public static int DESIRED_TAG_ID = 23; // <-- set the only tag to lock onto
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag;

    // ----------- Controls -----------
    // Right bumper = lock heading to the desired tag
    // Driver still controls

    // ----------- Heading PID (degrees) -----------
    // Bearing from ftcPose is in degrees; we keep PID in degrees for clarity.
    private final PIDHeadingController headingPid = new PIDHeadingController(0.020, 0.000, 0.001); // tune!
    private double lastPidTurn = 0.0;

    // Optional caps
    private static final double MAX_TURN_POWER = 0.6;

    @Override
    public void init() {
        // Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Typical mecanum directions (adjust if your robot moves wrong)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Vision
        initAprilTag();

        telemetry.addLine("AprilTag Lock (VisionPortal, OpMode)");
        telemetry.addData("Desired Tag ID", DESIRED_TAG_ID);
        telemetry.addLine("Hold RIGHT BUMPER to lock heading to tag.");
        telemetry.update();
    }

    @Override
    public void start() {
        headingPid.reset();
        lastPidTurn = 0.0;
    }

    @Override
    public void loop() {
        // Driver inputs (robot-centric)
        double fwd    = -gamepad1.left_stick_y;  // forward (+)
        double strafe = -gamepad1.left_stick_x;  // left (+)
        double rotate = -gamepad1.right_stick_x; // CCW (+)

        // Find the desired tag in the current detections
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

        // If right bumper held and tag is visible, override rotation with PID on bearing
        boolean lockRequested = gamepad1.right_bumper;
        double turnCmd;

        if (lockRequested && targetFound) {
            // Bearing is "how far left/right the tag is" in degrees (camera-centric)
            double bearingDeg = desiredTag.ftcPose.bearing; // +CCW, -CW
            // We want bearing -> 0 (centered)
            double pidOut = headingPid.update(0.0, bearingDeg);

            // Clamp turn power
            pidOut = Range.clip(pidOut, -MAX_TURN_POWER, MAX_TURN_POWER);
            lastPidTurn = pidOut;
            turnCmd = pidOut;
        } else {
            // Manual rotation
            turnCmd = rotate;
        }

        // Drive the robot (mecanum) with fwd/strafe from driver, rotation from either PID or stick
        moveRobot(fwd, strafe, turnCmd);

        // Telemetry
        telemetry.addData("Lock", lockRequested ? "ON (RB held)" : "OFF");
        telemetry.addData("Desired ID", DESIRED_TAG_ID);
        telemetry.addData("Tag Visible", targetFound);

        if (targetFound) {
            telemetry.addData("Tag Name", desiredTag.metadata.name);
            telemetry.addData("Range (in)", "%.1f", desiredTag.ftcPose.range);
            telemetry.addData("Bearing (deg)", "%.1f", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw (deg)", "%.1f", desiredTag.ftcPose.yaw);
            telemetry.addData("Elevation (deg)", "%.1f", desiredTag.ftcPose.elevation);
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

    /** Initialize the AprilTag processor and VisionPortal. */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagID(true)
                // You can tweak decimation for range vs FPS
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

    /**
     * Move robot according to desired axes motions (mecanum).
     * x: forward (+), y: strafe left (+), yaw: CCW (+)
     */
    private void moveRobot(double x, double y, double yaw) {
        double fl =  x - y - yaw;
        double fr =  x + y + yaw;
        double bl =  x + y - yaw;
        double br =  x - y + yaw;

        // Normalize
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
