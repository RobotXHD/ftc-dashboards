package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * Some additional constants for testing.
 */
@Config
public class pid {
    public static double pGyro = 0.025, iGyro = 0.0, dGyro = 0.069,kp=0.0,ki=0.0,kd=0.0;
    public static double p = 0.0, i = 0.0, d = 0.0, parm = 0.0, iarm = 0.0, darm = 0.0, pslider = 0.0, islider = 0.0, dslider = 0.0;
}