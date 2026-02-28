package org.firstinspires.ftc.teamcode.LeagueChampThursday;

public class S_Adaptive_Equations {
    public static double getFlywheelRPM (double distance_INCH){

        double rpm = distance_INCH * 14.7 + 2090;
        return Math.max(Math.min(rpm, 4200), 2500);
        // return Math.min((16.2 * distance_INCH) + 1700, 4000);

    }


    public static double getRampPos (double distance_INCH) {
        double pos = .006 * distance_INCH - 0.03333333;
        return Math.max(Math.min(pos - 0.04, .51), .3);
    }

}
