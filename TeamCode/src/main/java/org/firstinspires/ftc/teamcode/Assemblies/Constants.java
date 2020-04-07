package org.firstinspires.ftc.teamcode.Assemblies;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    //will become the place where we can tune and set values for ftc-dashboard @ competition

    //STONESCORER
    //Constants for init
    public static double initRotationServo = 0.70; //0.66
    public static double initFerrisServo = 0.32;
    public static double initClawServo = 1;

    // Servo constants for extakeOut method
    public static double clampClawServoPosition = 0.6;
    public static double placeFerrisServoPosition = 0.935;
    public static double backRotationServoPosition = 0.075; //0.1267

    // Constants for extakeIn
    public static double unclampClawServo = 1;
    public static double kickbackFerrisServo = 0.92;
    public static double frontRotationServo = 0.69;
    public static double intakeFerrisServo = 0.34;

    // For foundation
    public static double leftFoundationDown = 0.470;
    public static double rightFoundationDown = 0.08;
    public static double leftFoundationUp = 0.10;
    public static double rightFoundationUp = 0.515;

    // For vertical motor powers
    //TODO
    public static double mtrVerticalStop1 = 1;
    public static double mtrVerticalStop2 = 0;
}
