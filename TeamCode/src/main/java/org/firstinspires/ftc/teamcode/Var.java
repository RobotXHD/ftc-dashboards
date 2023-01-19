package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class Var {
    public enum DetectionTypes {
        DAY,
        NIGHT
    }

    public static double Day_Hhigh = 90, Day_Shigh = 255, Day_Vhigh = 255, Day_Hlow = 40, Day_Slow = 0, Day_Vlow = 0;
    public static double Night_Hhigh = 80, Night_Shigh = 255, Night_Vhigh = 255, Night_Hlow = 40, Night_Slow = 50, Night_Vlow = 0;
    public static double kp = 0.000002, ki = 0, kd = 0.0002;
    public static int CV_kernel_pult_size = 5, Webcam_w = 640, Webcam_h = 480, CV_rect_x1 = 0, CV_rect_y1 = 0, CV_rect_x2 = 640, CV_rect_y2 = 480;
    public static DetectionTypes CV_detectionType = DetectionTypes.NIGHT;
    public static Scalar scalar = new Scalar(0,0,0);
}
