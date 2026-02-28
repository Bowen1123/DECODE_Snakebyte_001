package org.firstinspires.ftc.teamcode.LeagueChampThursday;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

public class StorePose {
    public static Pose2d init;
    public StorePose(Pose2d pass){
        init = pass;
    }

    public StorePose(){
    }

    public String getPose(){
        return init.position.x + ", " + init.position.y + ", " + init.heading.toDouble();
    }


    public static Action savePos(Pose2d pos) { init = pos; return new SavePos(); }
    public static class SavePos implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }
}
