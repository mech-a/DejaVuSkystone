package org.firstinspires.ftc.teamcode.Assemblies;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    //will become the place where we can tune and set values for ftc-dashboard @ competition

    //STONESCORER
    //Constants for init
    public static double initRotationServo = 0.66;
    public static double initFerrisServo = 0.32;
    public static double initClawServo = 1;
    public static double initLeftFoundationServo = 0;
    public static double initRightFoundationServo = 0;

    // Servo constants for extakeOut method
    public static double clampClawServoPosition = 0.643;
    public static double placeFerrisServoPosition = 0.85;
    public static double backRotationServoPosition = 28.8/270 +.02;

    // Constants for extakeIn
    public static double unclampClawServo = 1;
    public static double kickbackFerrisServo = 0.92;
    public static double frontRotationServo = 0.643+(28.8/270);
    public static double intakeFerrisServo = 0.31; 

    // For foundation
    public static double leftFoundationDown = 0.72; //these are all probably old and useless
    public static double rightFoundationDown = 0.15;
    public static double leftFoundationUp = 0.10;
    public static double rightFoundationUp = 0.77;

    // For vertical motor powers
    public static double mtrVerticalStop1 = 1;
    public static double mtrVerticalStop2 = 0;
}
