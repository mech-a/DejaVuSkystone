package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Sensors;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is an example of a more complex path to really test the tuning.
 */

// CASE A: Next to wall
//PLEASE CONVERT TO RADIANS!!!!!!!!!
@Config
@Autonomous(group = "drive")
public class TwoStoneRedMT extends LinearOpMode {
    StoneScorer ss = new StoneScorer(this);
    //Sensors s = new Sensors(this);

    Sensors.SkyStoneLocation skyStoneLocation;

    public static double standardHeading = 0;
    public static double startingX = 0, startingY = 0;
    public static double skystoneLeftX = 40, skystoneCenterX = 40, skystoneRightX = 47.5;
    public static double skystoneLeftY = 12, skystoneCenterY = -8, skystoneRightY = -1;
    public static double distanceForwardToPickUpStone = 17.5;
    public static double distanceForwardToPickUpStoneRight = 7;
    public static double distanceForwardToPickUpStoneCenter = 6;
    public static double distanceForwardToPickUpStoneLeft = 10;
    public static double pulloutX = -30, pulloutY = 35, pulloutHeading = -90;
    public static double centerAngle = 45;
    public static double leftAngle = -45;
    public static double caseRightAngle = -90;
    public static double distanceStrafeLeftForFoundationSide = 55;
    public static double headingForStoneDrop = 90;
    //public static double distanceBackToPark = 25;

    public static double rotationBias = 12;

    public static double foundationRightX = 8;
    public static double foundationRightY = 28;
    public static double foundationHeading = 100;

    public static long sleepFromExtakeOutToExtakeIn = 1000, sleepFromExtakeInToIntakeIn = 1000;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized d = new SampleMecanumDriveREVOptimized(hardwareMap);

        ss.init(hardwareMap);
        //s.init(hardwareMap);

        //skyStoneLocation = s.findSkystone();

        waitForStart();

        //TODO reimpl.
        skyStoneLocation = Sensors.SkyStoneLocation.RIGHT;


        if (isStopRequested()) return;

        //starting at -35, 60
        d.setPoseEstimate(new Pose2d(startingX, startingY, standardHeading));

        switch(skyStoneLocation) {
            case LEFT:

                ExtakeThreadLeft etl = new ExtakeThreadLeft();
                etl.start();

                // spline to first left stone
                d.followTrajectorySync(
                        d.trajectoryBuilder().lineTo(new Vector2d(skystoneLeftX, strafeConvert(skystoneLeftY)),
                                new LinearInterpolator(Math.toRadians(standardHeading), Math.toRadians(leftAngle+25)))
                                .build());

//                ss.extakeIn();
//                ss.intake(-0.75);

                // go forward to pick up left stone
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(distanceForwardToPickUpStoneLeft)
                                .build()
                );

//                ss.intake(0);
//                ss.clampStone();

                // rotate to straighten out robot
                d.turnSync(Math.toRadians(24));//27.5

                // strafe to the left to avoid bridge
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .strafeLeft(strafeConvert(14.5))//12
                                .build()
                );

                // travel forward to foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(72.5)
                                .build()
                );

                // rotate 90 to face foundation
                d.turnSync(Math.toRadians(91));

                // back up against foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(7.5)
                                .build()
                );

                // hook foundation
                ss.hookFoundation();
                sleep(500);

                d.setPoseEstimate(new Pose2d(0, 0, 0));

                // spline to turn foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .lineTo(new Vector2d(foundationRightX, foundationRightY), new LinearInterpolator(0, Math.toRadians(foundationHeading + rotationBias-2.5)))
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

                // travel forward to second stone
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(80)// used to be 72
                                .build()
                );

                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .strafeLeft(strafeConvert(21))
                                .build()
                );

//                ss.extakeIn();
//                ss.intake(-0.75);

                // move forward to intake stone
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(8)//6.5
                                .build()
                );

//                sleep(1000);
//                ss.intake(0);
//                ss.clampStone();

                // strafe right to avoid bridge, back to get to foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .strafeRight(strafeConvert(18))
                                .build()
                );

                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(87)//86
                                .build()
                );

//                ss.extakeOutPartial();
//                sleep(1000);
//                ss.dropStone();
//                sleep(500);
//                ss.extakeIn();
//                sleep(1000);

                // park under bridge
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(40)
                                .build()
                );

                etl.interrupt();

                break;

            case CENTER:

                ExtakeThreadCenter etc = new ExtakeThreadCenter();
                etc.start();

                // spline to first center stone
                d.followTrajectorySync(
                        d.trajectoryBuilder().lineTo(new Vector2d(skystoneCenterX, strafeConvert(skystoneCenterY)),
                                new LinearInterpolator(Math.toRadians(standardHeading), Math.toRadians(centerAngle-20)))
                                .build());

                // move forward to intake block
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(distanceForwardToPickUpStoneCenter)
                                .build()
                );

//                ss.intake(0);
//                ss.clampStone();

                // strafe right to avoid bridge
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .strafeRight(21)
                                .build()
                );

                d.setPoseEstimate(new Pose2d(0, 0, 0));

                // turn 30 degrees to straighten out the robot
                d.turnSync(Math.toRadians(-27));

                // back up to the foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(78.5)
                                .build()
                );

                // turn 90 degrees to face foundation
                d.turnSync(Math.toRadians(-90));

                // back up to prepare to hook foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(6.5)
                                .build()
                );

                // hook foundation
                ss.hookFoundation();

                sleep(500);

                d.setPoseEstimate(new Pose2d(0, 0, 0));

                // spline to turn foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .lineTo(new Vector2d(foundationRightX, foundationRightY), new LinearInterpolator(0, Math.toRadians(foundationHeading + rotationBias-2.5)))
                                .build()
                );

                // unhook foundation
                ss.unhookFoundation();

                // push foundation into the wall
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

                // second stone process starts

                d.setPoseEstimate(new Pose2d(0, 0, 0));

                // move forward to the stones
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(82)
                                .strafeLeft(strafeConvert(22))
                                .build()
                );

//                ss.extakeIn();
//                ss.intake(-0.75);

                // move forward to intake second stone
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(8)
                                .build()
                );

                sleep(500);

//                ss.intake(0);
//                ss.clampStone();

                // strafe right to avoid bridge
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .strafeRight(strafeConvert(15))
                                .build()
                );

                // move back to foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(93)
                                .build()
                );

//                ss.extakeOutPartial();
//                sleep(1000);
//                ss.dropStone();
//                sleep(500);
//                ss.extakeIn();
//                sleep(1000);

                // move forward to park under bridge
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(40)
                                .build()
                );

                etc.interrupt();

                break;

            case RIGHT:

                ExtakeThreadRight etr = new ExtakeThreadRight();
                etr.start();

                // spline to get stone
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .lineTo(new Vector2d(skystoneRightX, strafeConvert(skystoneRightY)),
                                        new LinearInterpolator(Math.toRadians(standardHeading), Math.toRadians(caseRightAngle)))
                                .build());

//                ss.extakeIn();
//                ss.intake(-0.75);

                // go forward to pick up stone
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(distanceForwardToPickUpStoneRight)
                                .build()
                );

                // block picked up
//                ss.intake(0);
//                ss.clampStone();

                // moving left to avoid bridge
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .strafeLeft(strafeConvert(16))
                                .build()
                );

                // move back to foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(79)
                                .build()
                );

                // turn 90 degrees to have back of robot face foundation
                d.turnSync(Math.toRadians(90));

                // move backward to align back of robot with foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(7)
                                .build()
                );

                ss.hookFoundation();

                sleep(500);

                d.setPoseEstimate(new Pose2d(0, 0, 0));

                // spline to turn foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .lineTo(new Vector2d(foundationRightX, foundationRightY), new LinearInterpolator(0, Math.toRadians(foundationHeading + rotationBias-2.5)))
                                .build()
                );

                ss.unhookFoundation();

                // push foundation back into wall
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(24)
                                .build()
                );

                // extake out the first stone
//                ss.extakeOutPartial();
//                sleep(500);
//                ss.dropStone();
//                sleep(500);
//                ss.extakeIn();
//                sleep(1000);

                // second stone process starts

                d.setPoseEstimate(new Pose2d(0, 0, 0));

                // move forward to get second stone
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(96)
                                .build()
                );

                // move right to align with stone
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                //.strafeLeft(strafeConvert(2))
                                .strafeRight(strafeConvert(17))
                                .build()
                );

                // intake the block
//                ss.extakeIn();
//                ss.intake(-0.75);

                // move forward to intake the block
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(6)
                                .build()
                );


                // block picked up
//                ss.intake(0);
//                ss.clampStone();

                // move left to avoid bridge
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .strafeLeft(strafeConvert(13))
                                .build()
                );

                // move back to go to foundation
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .back(100)
                                .build()
                );

                // extake second block
//                ss.extakeOutPartial();
//                sleep(1000);
//                ss.dropStone();
//                sleep(500);
//                ss.extakeIn();
//                sleep(1000);

                // move forward to park under the bridge
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(42)
                                .build()
                );

                etr.interrupt();

                break;
        }
    }

    public static double strafeConvert(double distance) {
        return (1.2 * distance + 3.53);
    }

    private class ExtakeThreadLeft extends Thread {
        public ExtakeThreadLeft() {
            this.setName("ExtakeThreadLeft");
        }

        public void run() {
            try
            {
                while(!isInterrupted()) {
                    // spline to first skystone
                    sleep(2000);
                    // go forward to intake first block
                    ss.extakeIn();
                    ss.intake(-0.75);
                    sleep(1000);
                    ss.intake(0);
                    ss.clampStone();
                    // back up to foundation side of field
                    // rotate 90 to face foundation
                    sleep(9000);
                    // hook foundation
                    // turn foundation
                    // unhook foundation

                    // drop first stone onto foundation
                    ss.extakeOutPartial();
                    sleep(1500);
                    ss.dropStone();
                    sleep(200);
                    ss.extakeIn();
                    sleep(500);

                    // go to get second block
                    sleep(4000);

                    // go forward to intake second block
                    ss.intake(-0.75);
                    sleep(2000);
                    ss.intake(0);

                    // go back to foundation
                    sleep(2000);

                    // drop second stone onto foundation
                    ss.extakeOutPartial();
                    sleep(500);
                    ss.dropStone();
                    sleep(200);
                    ss.extakeIn();
                    sleep(500);

                    // park under the bridge
                }
            }
            catch (InterruptedException e)
            {
                ss.intake(0);
                ss.extakeIn();
            }

        }
    }

    private class ExtakeThreadCenter extends Thread {
        public ExtakeThreadCenter() {
            this.setName("ExtakeThreadCenter");
        }

        public void run() {
            try
            {
                while(!isInterrupted()) {
                    // spline to first skystone
                    sleep(2000);
                    // go forward to intake first block
                    ss.extakeIn();
                    ss.intake(-0.75);
                    sleep(1500);
                    ss.intake(0);
                    ss.clampStone();
                    // back up to foundation side of field
                    // rotate 90 to face foundation
                    sleep(7500);
                    // hook foundation
                    // turn foundation
                    // unhook foundation

                    // drop first stone onto foundation
                    ss.extakeOutPartial();
                    sleep(1500);
                    ss.dropStone();
                    sleep(200);
                    ss.extakeIn();
                    sleep(500);

                    // go to get second block
                    sleep(4000);

                    // go forward to intake second block
                    ss.intake(-0.75);
                    sleep(1000);
                    ss.intake(0);

                    // go back to foundation
                    sleep(3500);

                    // drop second stone onto foundation
                    ss.extakeOutPartial();
                    sleep(500);
                    ss.dropStone();
                    sleep(200);
                    ss.extakeIn();
                    sleep(500);

                    // park under the bridge
                }
            }
            catch (InterruptedException e)
            {
                ss.intake(0);
                ss.extakeIn();
            }
        }
    }

    private class ExtakeThreadRight extends Thread {
        public ExtakeThreadRight() {
            this.setName("ExtakeThreadRight");
        }

        public void run() {
            try
            {
                while(!isInterrupted()) {
                    // spline to first skystone
                    sleep(2500);
                    // go forward to intake first block
                    ss.extakeIn();
                    ss.intake(-0.75);
                    sleep(1000);
                    ss.intake(0);
                    ss.clampStone();
                    // back up to foundation side of field
                    // rotate 90 to face foundation
                    sleep(7000);
                    // hook foundation
                    // turn foundation
                    // unhook foundation

                    // drop first stone onto foundation
                    ss.extakeOutPartial();
                    sleep(1500);
                    ss.dropStone();
                    sleep(200);
                    ss.extakeIn();
                    sleep(500);

                    // go to get second block
                    sleep(5000);

                    // go forward to intake second block
                    ss.intake(-0.75);
                    sleep(1000);
                    ss.intake(0);

                    // go back to foundation
                    sleep(3000);

                    // drop second stone onto foundation
                    ss.extakeOutPartial();
                    sleep(1000);
                    ss.dropStone();
                    sleep(200);
                    ss.extakeIn();
                    sleep(500);

                    // park under the bridge
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