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
public class OneStoneRed extends LinearOpMode {
    StoneScorer ss = new StoneScorer(this);
    Sensors s = new Sensors(this);

    Sensors.SkyStoneLocation skyStoneLocation;

    public static double standardHeading = 0;
    public static double startingX = 0, startingY = 0;
    public static double skystoneLeftX = 47.5, skystoneCenterX = 40, skystoneRightX = 43;
    public static double skystoneLeftY = -8, skystoneCenterY = -13, skystoneRightY = 6.5;

    public static double distanceForwardToPickUpStoneRight = 10;
    public static double distanceForwardToPickUpStoneCenter = 6;
    public static double distanceForwardToPickUpStoneLeft = 8;

    public static double centerAngle = 65;
    public static double leftAngle = 90;
    public static double caseRightAngle = -45;

    public static double distanceStrafeLeftForFoundationSide = 55;
    public static double headingForStoneDrop = 90;
    //public static double distanceBackToPark = 25;

    public static double rotationBias = 10;

    public static double foundationTurnX = 8;
    public static double foundationTurnY = -28;
    public static double foundationHeading = -100;

    public static long sleepFromExtakeOutToExtakeIn = 1000, sleepFromExtakeInToIntakeIn = 1000;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREV d = new SampleMecanumDriveREV(hardwareMap);

        ss.init(hardwareMap);
        s.init(hardwareMap);

        skyStoneLocation = skyStoneLocation.LEFT;

        sleep(750);


        waitForStart();

        if (isStopRequested()) return;

        //starting at -35, 60
        d.setPoseEstimate(new Pose2d(startingX, startingY, standardHeading));

        switch(skyStoneLocation) {
            case LEFT:
                //s.shutdown();

                // spline to first left stone
                d.followTrajectorySync(
                        d.trajectoryBuilder().lineTo(new Vector2d(skystoneLeftX, strafeConvert(skystoneLeftY)),
                                new LinearInterpolator(Math.toRadians(standardHeading), Math.toRadians(leftAngle)))
                                .build());

                ss.extakeIn();
                ss.intake(-0.75);


                // go forward to pick up left stone
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(distanceForwardToPickUpStoneLeft)
                                .build()
                );

                sleep(1000);

                ss.intake(0);
                ss.clampStone();

                // strafe to the left to avoid bridge
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .strafeLeft(strafeConvert(16.5))
                                .build()
                );

                // travel backwards to foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(83)
                                .build()
                );

                // rotate 90 to face foundation
                d.turnSync(Math.toRadians(90));

                // back up against foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(9.5)
                                .build()
                );

                // hook foundation
                ss.hookFoundation();
                sleep(500);

                d.setPoseEstimate(new Pose2d(0, 0, 0));

                // spline to turn foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .lineTo(new Vector2d(foundationTurnX, foundationTurnY), new LinearInterpolator(0, Math.toRadians(foundationHeading - rotationBias)))
                                .build()
                );

                // unhook foundation
                ss.unhookFoundation();

                // push foundation into the wall
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(24)// used to be 26
                                .build()
                );

                ss.extakeOutPartial();
                sleep(500);
                ss.dropStone();
                sleep(500);
                ss.extakeIn();
                sleep(1000);

                // park under bridge
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(40)
                                .build()
                );


                break;

            case CENTER:
                s.shutdown();

                // spline to first center stone
                d.followTrajectorySync(
                        d.trajectoryBuilder().lineTo(new Vector2d(skystoneCenterX, strafeConvert(skystoneCenterY)),
                                new LinearInterpolator(Math.toRadians(standardHeading), Math.toRadians(centerAngle)))
                                .build());

                // move forward to intake block
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(distanceForwardToPickUpStoneCenter)
                                .build()
                );

                ss.intake(0);
                ss.clampStone();

                // strafe right to avoid bridge
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .strafeLeft(18)
                                .build()
                );

                d.setPoseEstimate(new Pose2d(0, 0, 0));

                // turn to straighten out the robot
                d.turnSync(Math.toRadians(30));

                // back up to the foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(82.5)
                                .build()
                );

                // turn 90 degrees to face foundation
                d.turnSync(Math.toRadians(90));

                // back up to prepare to hook foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(9.5)
                                .build()
                );

                // hook foundation
                ss.hookFoundation();

                sleep(500);

                d.setPoseEstimate(new Pose2d(0, 0, 0));

                // spline to turn foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .lineTo(new Vector2d(foundationTurnX, foundationTurnY), new LinearInterpolator(0, Math.toRadians(foundationHeading - rotationBias)))
                                .build()
                );

                // unhook foundation
                ss.unhookFoundation();

                // push foundation into the wall
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(22)// used to be 26
                                .build()
                );

                ss.extakeOutPartial();
                sleep(500);
                ss.dropStone();
                sleep(500);
                ss.extakeIn();

                // move forward to park under bridge
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(40)
                                .build()
                );


                break;

            case RIGHT:
                s.shutdown();


                // spline to get first right stone
                d.followTrajectorySync(
                        d.trajectoryBuilder().lineTo(new Vector2d(skystoneRightX, strafeConvert(skystoneRightY)),
                                new LinearInterpolator(Math.toRadians(standardHeading), Math.toRadians(-70)))
                                .build());

                ss.extakeIn();
                ss.intake(-0.75);

                // go forward to pick up right stone
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(distanceForwardToPickUpStoneRight)
                                .build()
                );

                ss.intake(0);
                ss.clampStone();

                // rotate to straighten out robot
                d.turnSync(Math.toRadians(- 20 - 3));//27.5

                // strafe to the right to avoid bridge
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .strafeRight(strafeConvert(13.25))//12
                                .build()
                );

                // travel forward to foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(70)// 74.5
                                .build()
                );

                // rotate 90 to face foundation
                d.turnSync(Math.toRadians(-90)); //-93

                // back up against foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(11)
                                .build()
                );

                // hook foundation
                ss.hookFoundation();
                sleep(500);

                d.setPoseEstimate(new Pose2d(0, 0, 0));

                // spline to turn foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .lineTo(new Vector2d(foundationTurnX, foundationTurnY), new LinearInterpolator(0, Math.toRadians(foundationHeading-rotationBias)))
                                .build()
                );

                // unhook foundation
                ss.unhookFoundation();

                // push foundation into the wall
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(25)// used to be 26
                                .build()
                );

                ss.extakeOutPartial();
                sleep(500);
                ss.dropStone();
                sleep(500);
                ss.extakeIn();
                sleep(1000);

                // park under bridge
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(40)
                                .build()
                );

                break;
        }
    }

    public static double strafeConvert(double distance) {
        return (1.2 * distance + 3.53);
    }
}