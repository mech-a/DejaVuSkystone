package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Sensors;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

// CASE A: Next to wall
//PLEASE CONVERT TO RADIANS!!!!!!!!!
@Config
@Autonomous(group = "drive")
public class JustRedFoundation extends LinearOpMode {
    StoneScorer ss = new StoneScorer(this);

    Sensors.SkyStoneLocation skyStoneLocation;

    public static double standardHeading = 0;
    public static double startingX = 0, startingY = 0;
    public static double skystoneLeftX = 47.5, skystoneCenterX = 40, skystoneRightX = 43;
    public static double skystoneLeftY = -8, skystoneCenterY = -13, skystoneRightY = 6.5;

    public static double distanceForwardToPickUpStoneRight = 10;
    public static double distanceForwardToPickUpStoneCenter = 6;
    public static double distanceForwardToPickUpStoneLeft = 7;

    public static double centerAngle = 65;
    public static double leftAngle = 90;
    public static double caseRightAngle = -45;

    public static double distanceStrafeLeftForFoundationSide = 55;
    public static double headingForStoneDrop = 90;
    //public static double distanceBackToPark = 25;

    public static double rotationBias = 12;

    public static double foundationTurnX = 8;
    public static double foundationTurnY = -28;
    public static double foundationHeading = -100;

    public static long sleepFromExtakeOutToExtakeIn = 1000, sleepFromExtakeInToIntakeIn = 1000;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREV d = new SampleMecanumDriveREV(hardwareMap);

        ss.init(hardwareMap);

        waitForStart();

        //TODO reimpl.

        if (isStopRequested()) return;

        //starting at -35, 60
        d.setPoseEstimate(new Pose2d(startingX, startingY, standardHeading));

        // travel forward to foundation
        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .back(28)
                        .build()
        );

        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .strafeLeft(13.5)
                        .build()
        );

        // rotate 90 to face foundation
        //d.turnSync(Math.toRadians(180));

        // back up against foundation
        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .back(5)
                        .build()
        );

        // hook foundation
        ss.hookFoundation();
        sleep(500);

        d.setPoseEstimate(new Pose2d(0, 0, 0));

        // spline to turn foundation
        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .lineTo(new Vector2d(foundationTurnX, foundationTurnY), new LinearInterpolator(0, Math.toRadians(-110)))
                        .build()
        );


        // unhook foundation
        ss.unhookFoundation();

        // push foundation back into wall
        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .back(24)
                        .build()
        );

//                ss.extakeOutPartial();
//                sleep(500);
//                ss.dropStone();
//                sleep(500);
//                ss.extakeIn();
//                sleep(1000);

        // path for getting second stone begins

        d.setPoseEstimate(new Pose2d(0, 0, 0));

        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .strafeLeft(30)// used to be 72
                        .build()
        );

        // travel forward to second stone
        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .forward(44)// used to be 72
                        .build()
        );
    }

    public static double strafeConvert(double distance) {
        return (1.2 * distance + 3.53);
    }

}