package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.RRMergedDrivetrain;
import org.firstinspires.ftc.teamcode.Assemblies.Sensors;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.yaml.snakeyaml.scanner.Constant;

import java.util.Vector;

/*
 * This is an example of a more complex path to really test the tuning.
 */

// CASE A: Next to wall
//PLEASE CONVERT TO RADIANS!!!!!!!!!
@Config
@Autonomous(group = "drive")
public class Case1A extends LinearOpMode {
    StoneScorer ss = new StoneScorer(this);
    //Sensors s = new Sensors(this);
    Sensors.SkyStoneLocation skyStoneLocation;

    public static double standardHeading = 0;
    public static double startingX = 0, startingY = 0;
    public static double skystoneLeftX = 28, skystoneCenterX, skystoneRightX = 46;
    public static double skystoneLeftY = 23, skystoneCenterY = 14, skystoneRightY = 1;
    public static double skystoneRightX2 = 28, skystoneRightY2 = 5;
    public static double distanceForwardToPickUpStone = 17.5;
    public static double distanceForwardToPickUpStoneRight = 7;
    public static double pulloutX = -30, pulloutY = 35, pulloutHeading = -90;
    public static double angle = -45;
    public static double caseRightAngle = -90;
    public static double distanceStrafeLeftForFoundationSide = 55;
    public static double headingForStoneDrop = 90;
    //public static double distanceBackToPark = 25;

    public static double rotationBias = 6.5;

    public static double foundationRightX = -88;
    public static double foundationRightY = 9;
    public static double foundationHeading = -90;

    public static long sleepFromExtakeOutToExtakeIn = 1000, sleepFromExtakeInToIntakeIn = 1000;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized d = new SampleMecanumDriveREVOptimized(hardwareMap);

        ss.init(hardwareMap);
        //s.init();

        waitForStart();

        //TODO reimpl.
        //skyStoneLocation = s.findSkystone();
        skyStoneLocation = Sensors.SkyStoneLocation.RIGHT;

        if (isStopRequested()) return;

        //starting at -35, 60
        d.setPoseEstimate(new Pose2d(startingX, startingY, standardHeading));

        switch(skyStoneLocation) {
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

                sleep(1000);

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

                ss.extakeIn();

                ss.intake(-0.75);

                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .forward(distanceForwardToPickUpStoneRight)
                                .build()
                );

                sleep(1000);

                //block picked up
                ss.intake(0);


                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                //y used to be 50, too far
                                //.lineTo(new Vector2d(skystonePositionX, 10), new ConstantInterpolator(-90))
                                .back(distanceForwardToPickUpStoneRight)
                                .strafeRight(24)
                                .build());

                d.setPoseEstimate(new Pose2d(0, 0, 0));

                //strafe right and cross under bridge
                d.followTrajectorySync(
                        d.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(foundationRightX, foundationRightY, Math.toRadians(foundationHeading)))
                                .build()
                );
        }


        //TODO come up with better way to pull out, probably can go backwards, rn will interfere with blocks
        //pull out of area
//        d.followTrajectorySync(
//                d.trajectoryBuilder()
//                        .reverse()
//                        .splineTo(new Pose2d(pulloutX, pulloutY, pulloutHeading))
//                        //.back(distanceForwardToPickUpStone)
//                        .build()
//        );

//        telemetry.addData("stat", "after pullout");
//
//        // move over to foundation side
//        d.followTrajectorySync(
//                d.trajectoryBuilder()
//                        .reverse()
//                        //.splineTo(new Pose2d(-5, 38, -180), new LinearInterpolator(-90, -180))
//                        //.strafeLeft(distanceStrafeLeftForFoundationSide)
//                        //.reverse()
//                        .lineTo(new Vector2d(skystonePositionX, 33))
//                        .build()
//        );
//
//
//
//
//
//        d.turnSync(Math.toRadians(-90-rotationBias));
//
//        d.followTrajectorySync(
//                d.trajectoryBuilder()
//                        .reverse()
//                        .lineTo(new Vector2d(20, 40))
//                        .build()
//        );
//
//        telemetry.addData("Stat", "after something");
//        telemetry.addData("Heading", d.getPoseEstimate().getHeading());
//        telemetry.update();
//
//        //d.turnSync(Math.toRadians(180)-d.getPoseEstimate().getHeading());
//
//        sleep(1000);
//
//        //d.getPoseEstimate().getHeading();
//
//        //d.turnSync(Math.toRadians(headingForStoneDrop));
//
//        //TODO extake
//        ss.extakeOut();
//        sleep(500);
//        ss.dropStone();
//        sleep(500);
//        ss.extakeIn();
//        sleep(1000);
//
//        //park under the bridge
//        d.followTrajectorySync(
//                d.trajectoryBuilder()
//                        .lineTo(new Vector2d(0, pulloutY), new ConstantInterpolator(90)
//                        )
//                        //.strafeLeft(25)
//                        .build()
//                        );
//        //extake
//        //TODO ext.
//        ss.setBlock(-10, -10);
/*
        if (skyStoneLocation == Sensors.SkyStoneLocation.LEFT) {
            d.followTrajectorySync(
                    d.trajectoryBuilder()
                            .splineTo(new Pose2d(-52, -30, 0))
                            .build() );
        } else if (skyStoneLocation == Sensors.SkyStoneLocation.CENTER) {
            d.followTrajectorySync(
                    d.trajectoryBuilder()
                            .splineTo(new Pose2d(-60, -30, 0))
                            .build() );
        } else if (skyStoneLocation == Sensors.SkyStoneLocation.RIGHT) {
            d.followTrajectorySync(
                    d.trajectoryBuilder()
                            .splineTo(new Pose2d(-68, -30, 0))
                            .build() );
        }

        //intake
        ss.intake(0.75); //TODO: ALL STONE SCORER FUNCTIONS NEED TO BE CHANGED
*/
//        d.followTrajectorySync(
//                d.trajectoryBuilder()
//                        //.splineTo(new Pose2d(20, -50, 0))
//                        .lineTo(new Vector2d(20, -50))
//                        .build()
//        );
//
//        //extake
//      //  ss.setBlock(-10, -10);
//
//        //parking next to wall
//        d.followTrajectorySync(
//                d.trajectoryBuilder()
//                        .splineTo(new Pose2d(0, -60, 0))
//                        .build()
//        );
    }

    public static double strafeConvert(double distance) {
        return (1.2 * distance + 3.53);
    }
}

