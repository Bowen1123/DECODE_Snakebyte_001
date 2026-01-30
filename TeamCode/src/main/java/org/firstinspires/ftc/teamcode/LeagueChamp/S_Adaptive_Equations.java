package org.firstinspires.ftc.teamcode.LeagueChamp;

public class S_Adaptive_Equations {
    public static double getFlywheelRPM (double distance_INCH){

        return Math.min((17 * distance_INCH) + 2160, 5500);

    }


    public static double getRampPos (double distance_INCH) {
        return 0;
    }

}
