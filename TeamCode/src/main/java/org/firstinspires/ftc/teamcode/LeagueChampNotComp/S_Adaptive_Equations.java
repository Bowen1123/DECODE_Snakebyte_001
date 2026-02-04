package org.firstinspires.ftc.teamcode.LeagueChampNotComp;

public class S_Adaptive_Equations {
    public static double getFlywheelRPM (double distance_INCH){

        return Math.min((17.5 * distance_INCH) + 2350, 5500);

    }


    public static double getRampPos (double distance_INCH) {
        return Math.min(Math.max(distance_INCH / 100, 0), 1);
    }

}
