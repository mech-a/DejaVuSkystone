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
  //  public static double initLeftFoundationServo = 0.10;
    //public static double initRightFoundationServo = 0.77;

    // Servo constants for extakeOut method
    public static double clampClawServoPosition = 0.6;
    public static double placeFerrisServoPosition = 0.87;
    public static double backRotationServoPosition = 28.8/270 +.02; //0.1267

    // Constants for extakeIn
    public static double unclampClawServo = 1;
    public static double kickbackFerrisServo = 0.92;
    public static double frontRotationServo = 0.745;
    public static double intakeFerrisServo = 0.34;

    // For foundation
    public static double leftFoundationDown = 0.72; //these are all probably old and useless
    public static double rightFoundationDown = 0.08;
    public static double leftFoundationUp = 0.10;
    public static double rightFoundationUp = 0.515;

    // For vertical motor powers
    //TODO
    public static double mtrVerticalStop1 = 1;
    public static double mtrVerticalStop2 = 0;
}
