package org.firstinspires.ftc.teamcode.TrollBot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;

@TeleOp
public class Auto_Position_Finder extends LinearOpMode {

    private ArrayList<Double> xlist;
    private ArrayList<Double> ylist;
    private ArrayList<Double> headinglist;

    private ArrayList<Pose2d> positions;
    private ArrayList<Action> pathing;

    private double x;
    private double y;
    private double heading;

    private DcMotor frontRight, frontLeft, backLeft, backRight;
    private IMU imu;

    private Action driveBack;

    private double SPEED_MULTIPLIER = 0.8;

    @Override
    public void runOpMode() {
        waitForStart();

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        double xo = initialPose.position.x;
        double yo = initialPose.position.y;
        double headingo = initialPose.heading.toDouble();

        // Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft   = hardwareMap.get(DcMotor.class, "leftBack");
        backRight  = hardwareMap.get(DcMotor.class, "rightBack");

        /*frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);*/

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

        xlist = new ArrayList<>();
        ylist = new ArrayList<>();
        headinglist = new ArrayList<>();
        positions = new ArrayList<>();

        boolean showList = false;
        boolean prevAButton = false, prevYButton = false, prevBButtone = false, prevXButton = false, prevTriggers = false;

        while(opModeIsActive()){
            drive.updatePoseEstimate();

            Pose2d currentPose = drive.localizer.getPose();
            x = currentPose.position.x;
            y = currentPose.position.y;
            heading = currentPose.heading.toDouble();


            // Button A: Save Location
            if (gamepad1.a && !prevAButton){
                xlist.add(x);
                ylist.add(y);
                headinglist.add(heading);
                positions.add(currentPose);

            }

            if (showList){
                telemetry.addLine("\nX: " + xlist.toString());
                telemetry.addLine("\nY: " + ylist.toString());
                telemetry.addLine("\nHeading: " + headinglist.toString());
            }

            // Toggle show positions
            if (gamepad1.y && !prevYButton){
                showList = !showList;
            }


            // Generate pathing actions from positions
            Pose2d pastPos = initialPose, nextPos = null;
            if (gamepad1.b && !prevBButtone && !positions.isEmpty()){

                for (int i = 0; i < positions.size(); i++){

                    if (i == 0 && positions.size() > i+1){
                        nextPos = positions.get(i+1);

                        Action path = drive.actionBuilder(pastPos)
                                .setTangent(Math.atan((nextPos.position.y - y) / (nextPos.position.x - x)))
                                .splineTo(new Vector2d(nextPos.position.x, nextPos.position.x), nextPos.heading.toDouble())
                                .build();

                        pathing.add(path);
                    }

                    pastPos = positions.get(i);
                }
            }

            if (gamepad1.x && !prevXButton){
                if (!pathing.isEmpty()){
                    Action run = pathing.remove(0);
                    Actions.runBlocking(run);
                }
            }

            if (gamepad1.right_trigger > 0.5 && gamepad1.left_trigger > 0.5 && !prevTriggers){
                 driveBack = drive.actionBuilder(drive.localizer.getPose())
                         .setTangent(Math.atan((y-yo)/(x-xo)) - Math.toRadians(180))
                         .splineTo(new Vector2d(xo, yo), headingo)
                         .build();

                Actions.runBlocking(driveBack);
            }

            prevYButton = gamepad1.y;
            prevAButton = gamepad1.a;
            prevBButtone = gamepad1.b;
            prevXButton = gamepad1.x;
            prevTriggers = gamepad1.right_trigger > .5 && gamepad1.left_trigger > 0.5;

            telemetry.addLine("Position: ( " +  drive.localizer.getPose().position.x + " , "+ drive.localizer.getPose().position.y +" )" );
            telemetry.addLine("Heading (Degree):  " + Math.toDegrees(drive.localizer.getPose().heading.toDouble()));


            // ---------------- Driver inputs ----------------
            double driveY = -gamepad1.left_stick_y;  // forward/back
            double driveX =  -gamepad1.left_stick_x;  // left/right
            double driveTurn = -gamepad1.right_stick_x; // ccw/cw

            double xPower, yPower;

            double headingRad = Math.toRadians(getHeadingDeg());

            double fieldY = driveY;
            double fieldX = driveX;

            double robotY =  fieldY * Math.cos(headingRad) + fieldX * Math.sin(headingRad);
            double robotX = -fieldY * Math.sin(headingRad) + fieldX * Math.cos(headingRad);

            yPower = robotY;
            xPower = robotX;

            moveRobot(xPower, yPower, driveTurn);

            telemetry.update();
            telemetry.clear();
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

        frontLeft.setPower(fl * SPEED_MULTIPLIER);
        frontRight.setPower(fr * SPEED_MULTIPLIER);
        backLeft.setPower(bl * SPEED_MULTIPLIER);
        backRight.setPower(br * SPEED_MULTIPLIER);
    }
}
