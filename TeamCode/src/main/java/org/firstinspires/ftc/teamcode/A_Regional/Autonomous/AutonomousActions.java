package org.firstinspires.ftc.teamcode.A_Regional.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.A_Regional.C_Intake;
import org.firstinspires.ftc.teamcode.A_Regional.C_Shooter;
import org.firstinspires.ftc.teamcode.A_Regional.C_ShooterTurret;
import org.firstinspires.ftc.teamcode.A_Regional.C_Transfer;
import org.firstinspires.ftc.teamcode.A_Regional.D_BasicTurret;

public class AutonomousActions {

    private final C_Intake intake;
    private final C_Transfer transfer;
    private final C_ShooterTurret shooterTurret;
    private double distanceIn = 70;
    public AutonomousActions(C_Intake intake,
                             C_Transfer transfer,
                             C_ShooterTurret st) {
        this.intake = intake;
        this.transfer = transfer;
        this.shooterTurret = st;

    }

    public Action intakeIntake() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.intake();
                return false;
            }
        };
    }

    public Action intakeOuttake() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.outtake();
                return false;
            }
        };
    }

    public Action intakeDeactivate() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.deactivate();
                return false;
            }
        };
    }


    public Action transferIntake() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                transfer.intake();
                return false;
            }
        };
    }

    public Action transferOuttake() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                transfer.outtake();
                return false;
            }
        };
    }


    public Action transferDeactivate() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                transfer.deactivate();
                return false;
            }
        };
    }

    public Action int_tf_intake() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.intake();
                transfer.intake();
                return false;
            }
        };
    }

    public Action int_tf_outtake() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.outtake();
                transfer.outtake();
                return false;
            }
        };
    }

    public Action int_tf_deactivate() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.deactivate();
                transfer.deactivate();
                return false;
            }
        };
    }
    public Action setPipelineA(int pipes) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shooterTurret.setPipelines(pipes);
                return false;
            }
        };
    }

    public Action track() {
        return new Action() {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!started) {
                    started = true;
                    timer.reset();
                    shooterTurret.setTrackingEnabled(true);
                }

                // Update vision + run turret control
                shooterTurret.updateLimelight();
                boolean aimed = shooterTurret.turretLoop();

                boolean hasTarget = shooterTurret.hasTarget();

                packet.put("hasTarget: ", hasTarget);
                packet.put("aimed: ", aimed);


                // ✅ Finish ONLY when we have target AND are aimed
                if (hasTarget && aimed) {

                    double d = shooterTurret.getGroundDistanceInches();
                    if (!Double.isNaN(d) && d > 0.0) {
                        distanceIn = d;
                    } else {
                        distanceIn = 70.0;
                    }

                    shooterTurret.setTrackingEnabled(false);
                    shooterTurret.turretStop();
                    return false; // DONE
                }

                // ✅ Safety timeout (never hang auto)
                if (timer.seconds() >= 4) {
                    distanceIn = 70.0;

                    shooterTurret.setTrackingEnabled(false);
                    shooterTurret.turretStop();

                    return false; // DONE (fallback distance)
                }

                return true; // Keep running
            }
        };
    }

    public Action spinFlywheelHold() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shooterTurret.setShooterEnabled(true);
                shooterTurret.setDistanceInches(distanceIn);

                // Runs the internal PID and writes motors
                shooterTurret.shooterUpdate();

                packet.put("DistanceIn: ", distanceIn);
                packet.put("Speed: ", shooterTurret.getTargetRPM());

                return true; // hold for rest of auto
            }
        };
    }

    public Action spinFlywheelTemp() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shooterTurret.setShooterEnabled(true);
                shooterTurret.setDistanceInches(distanceIn);

                // Runs the internal PID and writes motors
                shooterTurret.shooterUpdate();

                packet.put("DistanceIn: ", distanceIn);
                packet.put("Speed: ", shooterTurret.getTargetRPM());

                return shooterTurret.getMeasuredRPM() < shooterTurret.getTargetRPM(); // hold for rest of auto
            }
        };
    }

    public Action stopTrack() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shooterTurret.setTrackingEnabled(false);
                shooterTurret.turretStop();
                return false;
            }
        };
    }
}