package org.firstinspires.ftc.teamcode.TrollBot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Disabled
@Config
@Autonomous(name = "Auto_Pathing", group = "Auto")
public class Auto_Pathing extends LinearOpMode {

    // AprilTag IDs
    public static int GPP = 21;
    public static int PGP = 22;
    public static int PPG = 23;

    public static double TAG_TIMEOUT = 2.5;
    public static boolean USE_WEBCAM = true;
    public static String WEBCAM_NAME = "Webcam 1";

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Integer detectedId = null;

    @Override
    public void runOpMode() {
        // -------- Initialize Vision --------
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .addProcessor(aprilTag);
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME));

        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        visionPortal = builder.build();


        // Pathing!
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Example placeholder paths (replace with real coordinates)
        TrajectoryActionBuilder pathGPPBuilt = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(35, 35), Math.toRadians(270));

        Action pathGPP = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(35, 35), Math.toRadians(270))
                .build();

        Action pathPGP = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(50, 45), Math.toRadians(270))
                .build();

        Action pathPPG = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(20, 45), Math.toRadians(270))
                .build();

        Action pathDefault = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(25, 50), Math.toRadians(270))
                .build();

        // -------- INIT LOOP --------
        while (!isStarted() && !isStopRequested()) {
            detectedId = null;
            for (AprilTagDetection d : aprilTag.getDetections()) {
                if (d.id == GPP || d.id == PGP || d.id == PPG) {
                    detectedId = d.id;
                    break;
                }
            }
            telemetry.addData("Detected Tag", detectedId == null ? "none" : detectedId);
            telemetry.update();
            sleep(20);
        }

        // -------- Post-Start Tag Scan --------
        if (detectedId == null && TAG_TIMEOUT > 0) {
            ElapsedTime timer = new ElapsedTime();
            while (opModeIsActive() && detectedId == null && timer.seconds() < TAG_TIMEOUT) {
                for (AprilTagDetection d : aprilTag.getDetections()) {
                    if (d.id == GPP || d.id == PGP || d.id == PPG) {
                        detectedId = d.id;
                        break;
                    }
                }
                telemetry.addData("Searching for DECODE tag...", "%.1f / %.1f sec",
                        timer.seconds(), TAG_TIMEOUT);
                telemetry.update();
                sleep(20);
            }
        }

        // -------- Run Auto Path --------
        telemetry.addLine("Starting AUTO");
        telemetry.addData("Detected ID", detectedId == null ? "NONE (Default)" : detectedId);
        telemetry.update();

        if (detectedId == null) {
            Actions.runBlocking(pathDefault);
        } else if (detectedId == GPP) {

        } else if (detectedId == PGP) {

        } else if (detectedId == PPG) {

        }


        telemetry.addLine("Auto complete.");
        telemetry.update();
        if (visionPortal != null) visionPortal.close();
    }
}