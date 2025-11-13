package org.firstinspires.ftc.teamcode.TrollBot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Disabled
@TeleOp
public class Teleop_Pathing extends LinearOpMode {

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

    private Action defaultPath, GPP_Path, PGP_Path, PPG_Path;

    @Override
    public void runOpMode() {
        waitForStart();
        // -------- Initialize Vision --------
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .addProcessor(aprilTag);
        if (USE_WEBCAM){
            builder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME));

        }
        else{
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        visionPortal = builder.build();

        // Pathing!
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        while (opModeIsActive()) {

            Action pathGPP = drive.actionBuilder(initialPose)
                    .splineToConstantHeading(new Vector2d(35, 35), Math.toRadians(270))
                    .build();

            Action pathPGP = drive.actionBuilder(initialPose)
                    .splineToConstantHeading(new Vector2d(50, 45), Math.toRadians(270))
                    .build();

            Action pathPPG = drive.actionBuilder(initialPose)
                    .splineToConstantHeading(new Vector2d(20, 45), Math.toRadians(270))
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


            // -------- Run Auto Path --------
            telemetry.addLine("Starting AUTO");
            telemetry.addData("Detected ID", detectedId == null ? "NONE (Default)" : detectedId);

            if (gamepad1.a) {
                Action pathDefault = drive.actionBuilder(initialPose)
                        .splineTo(new Vector2d(24, 48), Math.toRadians(90))
                        .build();

                Actions.runBlocking(pathDefault);
            }

            if (gamepad1.b){
                Action pathDefault2 = drive.actionBuilder(new Pose2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, Math.toRadians(0)))
                        .setTangent(Math.toRadians(-135))
                        .splineToSplineHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(90))
                        .build();

                Actions.runBlocking(pathDefault2);
            }
            //else if (detectedId == GPP){

            //}
//        else if (detectedId == PGP){
//
//        }
//        else if (detectedId == PPG){
//
//        }

            drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();

            telemetry.addLine("(X,Y):   " + "(" + currentPose.position.x + ", " + currentPose.position.y +")");
            telemetry.addLine("Auto complete.");
            telemetry.update();
        }

        if (visionPortal != null) visionPortal.close();
    }

    public void generatePathes(){


    }

}