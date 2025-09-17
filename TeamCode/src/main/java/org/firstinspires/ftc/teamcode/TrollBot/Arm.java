package org.firstinspires.ftc.teamcode.TrollBot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Config
@TeleOp
public class Arm{
    private DcMotor arm;


 // will finish later
    private static double p = 0, i = 0, d = 0, f = 0;
    public Arm() {



        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

}
