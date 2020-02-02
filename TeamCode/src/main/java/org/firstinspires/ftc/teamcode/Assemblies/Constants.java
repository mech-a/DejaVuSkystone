package org.firstinspires.ftc.teamcode.Assemblies;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    //will become the place where we can tune and set values for ftc-dashboard @ competition

    //STONESCORER
    //Constants for init
    public static double initRotationServo = 0.66;
    public static double initferrisServo = 0.32;
    public static double initClawServo = 1;
    public static double initFoundationServo = 0;

    // Servo constants for extakeOut method
    public static double extakeOutClawServoPosition = 0.643;
    public static double extakeOutFerrisServoPosition = 0.85;
    public static double extakeOutRotationServoPosition = 28.8/270 +.02;

    // Constants for extakeIn
    public static double extakeInClawServo = 1;
    public static double extakeInFerrisServo = 0.92;
    public static double extakeInRotationServo = 0.643+(28.8/270);
    public static double extakeInFerrisServo2 = 0.31;

    // For foundation
    public static double foundationDown = 0;
    public static double foundationUp = 0.65;

    // For vertical motor powers
    public static double mtrVerticalStop1 = 1;
    public static double mtrVerticalStop2 = 0;
}
