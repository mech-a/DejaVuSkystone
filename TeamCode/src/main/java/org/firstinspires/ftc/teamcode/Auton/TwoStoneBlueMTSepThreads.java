package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Sensors;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is an example of a more complex path to really test the tuning.
 */


@Disabled
@Autonomous(group = "drive")
public class TwoStoneBlueMTSepThreads extends LinearOpMode {
    StoneScorer ss = new StoneScorer(this);
    //Sensors s = new Sensors(this);
    Sensors.SkyStoneLocation skyStoneLocation;

    public static double standardHeading = 0;
    public static double startingX = 0, startingY = 0;
    public static double skystoneLeftX = 28, skystoneCenterX, skystoneRightX = 47.5;
    public static double skystoneLeftY = 23, skystoneCenterY = 14, skystoneRightY = 1;
    public static double distanceForwardToPickUpStone = 17.5;
    public static double distanceForwardToPickUpStoneRight = 7;
    public static double pulloutX = -30, pulloutY = 35, pulloutHeading = -90;
    public static double angle = -45;
    public static double caseRightAngle = -90;
    public static double distanceStrafeLeftForFoundationSide = 55;
    public static double headingForStoneDrop = 90;
    //public static double distanceBackToPark = 25;

    public static double rotationBias = 6.5;

    public static double foundationRightX = 12;
    public static double foundationRightY = 28;
    public static double foundationHeading = 100;

    public static long sleepFromExtakeOutToExtakeIn = 1000, sleepFromExtakeInToIntakeIn = 1000;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREV d = new SampleMecanumDriveREV(hardwareMap);

        ss.init(hardwareMap);
        //s.init();

        waitForStart();

        //TODO reimpl.
        //skyStoneLocation = s.findSkystone();

        skyStoneLocation = Sensors.SkyStoneLocation.RIGHT;

        if (isStopRequested()) return;

        //starting at -35, 60
        d.setPoseEstimate(new Pose2d(startingX, startingY, standardHeading));

        switch (skyStoneLocation) {
            case LEFT:
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .lineTo(new Vector2d(skystoneLeftX, strafeConvert(skystoneLeftY)),
                                        new LinearInterpolator(Math.toRadians(standardHeading), Math.toRadians(angle)))
                                .build());

                ss.extakeIn();

                //intake
                ss.intake(-0.75);

                //drives forward to pick up
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(distanceForwardToPickUpStone)
                                .build()
                );

                sleep(500);

                //block picked up
                ss.intake(0);

                ss.clampStone();

                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                //y used to be 50, too far
                                //.lineTo(new Vector2d(skystonePositionX, 10), new ConstantInterpolator(-90))
                                .back(distanceForwardToPickUpStone)
                                .build()
                );

                break;

            case CENTER:
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .lineTo(new Vector2d(skystoneCenterX, strafeConvert(skystoneCenterY)),
                                        new LinearInterpolator(Math.toRadians(standardHeading), Math.toRadians(angle)))
                                .build());

                ss.extakeIn();

                ss.intake(-0.75);

                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(distanceForwardToPickUpStone)
                                .build()
                );

                sleep(1000);

                //block picked up
                ss.intake(0);


                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                //y used to be 50, too far
                                //.lineTo(new Vector2d(skystonePositionX, 10), new ConstantInterpolator(-90))
                                .back(distanceForwardToPickUpStone)
                                .build()
                );

                break;

            case RIGHT:
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .lineTo(new Vector2d(skystoneRightX, strafeConvert(skystoneRightY)),
                                        new LinearInterpolator(Math.toRadians(standardHeading), Math.toRadians(caseRightAngle)))
                                .build());

                FirstStoneIntake fsIntake = new FirstStoneIntake();
                fsIntake.start();

                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(distanceForwardToPickUpStoneRight)
                                .build()
                );


                //block picked up
//                ss.intake(0);
//
//                ss.clampStone();

                //moving block to foundation side
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .strafeRight(strafeConvert(13.5))
                                .back(86)
                                .build()

                );

                fsIntake.interrupt();

                FirstStoneExtakeOut fsExOut = new FirstStoneExtakeOut();
                fsExOut.start();

                d.turnSync(Math.toRadians(-90));
                //strafe right and cross under bridge
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(13)
                                .build()
                );

                ss.hookFoundation();

                sleep(500);

                d.setPoseEstimate(new Pose2d(0, 0, 0));

                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .lineTo(new Vector2d(foundationRightX, foundationRightY), new LinearInterpolator(0, Math.toRadians(foundationHeading + rotationBias)))
                                .build()
                );

                ss.unhookFoundation();

                fsExOut.interrupt();

                FirstStoneRelease fsRelease = new FirstStoneRelease();
                fsRelease.start();

                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(20)
                                .build()
                );

//                ss.extakeOutPartial();
//                sleep(500);
//                ss.dropStone();
//                sleep(500);
//                ss.extakeIn();
//                sleep(1000);

                //testing starts

                fsRelease.interrupt();

                SecondStoneRelease ssRelease = new SecondStoneRelease();
                ssRelease.start();

                d.setPoseEstimate(new Pose2d(0, 0, 0));

                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                //.strafeLeft(strafeConvert(2))
                                .forward(88)
                                .strafeLeft(strafeConvert(25))
                                .build()
                );

//                ss.extakeIn();
//
//                ss.intake(-0.75);

                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(6)
                                .build()
                );


                //block picked up
//                ss.intake(0);
//
//                ss.clampStone();

                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .strafeRight(strafeConvert(13))
                                .back(102)
                                .build()
                );

                ssRelease.interrupt();

//                ss.extakeOutPartial();
//                sleep(1000);
//                ss.dropStone();
//                sleep(500);
//                ss.extakeIn();
//                sleep(1000);


                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(40)
                                .build()
                );

        }
    }

    public static double strafeConvert(double distance) {
        return (1.2 * distance + 3.53);
    }

    private class FirstStoneIntake extends Thread {

        public FirstStoneIntake() {
            this.setName("FirstStoneIntake");
        }

        public void run() {
            try
            {
                while(!isInterrupted()) {
                    // spline to first skystone
                    ss.extakeIn();
                    ss.intake(-0.75);
                    sleep(1000);
                    ss.intake(0);
                    ss.clampStone();
                    sleep(7000);
                }
            }
            catch (InterruptedException e)
            {
                ss.intake(0);
                ss.extakeIn();
                ss.clampStone();
            }
        }
    }

    private class FirstStoneExtakeOut extends Thread {

        public FirstStoneExtakeOut() {
            this.setName("FirstStoneExtakeOut");
        }

        public void run() {
            try
            {
                while(!isInterrupted()) {
                    // before this, grip foundation
                    ss.extakeOutPartial();
                    sleep(1500);
                }
            }
            catch (InterruptedException e)
            {
                ss.intake(0);
                ss.extakeIn();
            }
        }
    }

    private class FirstStoneRelease extends Thread {

        public FirstStoneRelease() { this.setName("FirstStoneRelease"); }

        public void run() {
            try
            {
                while(!isInterrupted()) {
                    // before this, release foundation
                    ss.dropStone();
                    sleep(200);
                    ss.extakeIn();
                    sleep(500);
                }
            }
            catch (InterruptedException e)
            {
                ss.intake(0);
                ss.extakeIn();
            }
        }
    }

    private class SecondStoneRelease extends Thread {

        public SecondStoneRelease() { this.setName("SecondStoneRelease"); }

        public void run() {
            try
            {
                while(!isInterrupted()) {
                    // before this, go forward to second block
                    ss.intake(-0.75);
                    sleep(1000);
                    ss.intake(0);

                    // go back to foundation
                    sleep(2000);

                    ss.extakeOutPartial();
                    sleep(500);
                    ss.dropStone();
                    sleep(200);
                    ss.extakeIn();
                    sleep(500);
                    // park
                }
            }
            catch (InterruptedException e)
            {
                ss.intake(0);
                ss.extakeIn();
            }
        }
    }



    private class ExtakeThread extends Thread {

        public ExtakeThread() {
            this.setName("ExtakeThread");
        }

        public void run() {
            try
            {
                while(!isInterrupted()) {
                    // spline to first skystone
                    sleep(2500);
                    ss.extakeIn();
                    ss.intake(-0.75);
                    sleep(1000);

                    // sleep, currently in the main thread
                    ss.intake(0);
                    ss.clampStone();

                    sleep(7000);
                    // hook foundation
                    // turn foundation
                    // sleep(1000);
                    // unhook foundation

                    ss.extakeOutPartial();
                    sleep(1500);
                    ss.dropStone();
                    sleep(200);
                    ss.extakeIn();
                    sleep(500);

                    // go to get second block
                    sleep(3500);

                    ss.intake(-0.75);
                    sleep(1000);
                    ss.intake(0);

                    // go back to foundation
                    sleep(2000);

                    ss.extakeOutPartial();
                    sleep(500);
                    ss.dropStone();
                    sleep(200);
                    ss.extakeIn();
                    sleep(500);
                    // park
                }
            }
            catch (InterruptedException e)
            {
                ss.intake(0);
                ss.extakeIn();
            }
        }
    }
}