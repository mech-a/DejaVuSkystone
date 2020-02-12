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
@Config
@Autonomous(group = "drive")
public class Case1A extends LinearOpMode {
    StoneScorer ss = new StoneScorer(this);
    //Sensors s = new Sensors(this);
    Sensors.SkyStoneLocation skyStoneLocation;

    public static double standardHeading = -90;
    public static double startingX = -32.75, startingY = 63.75;
    public static double skystoneLeftX = -36, skystoneCenterX = -35, skystoneRightX = -45;
    public static double skystoneY = 36;
    public static double distanceForwardToPickUpStone = 20;
    public static double pulloutX = -30, pulloutY = 35, pulloutHeading = -90;
    public static double distanceStrafeLeftForFoundationSide = 55;
    public static double headingForStoneDrop = 90;
    //public static double distanceBackToPark = 25;

    public static double rotationBias = 6.5;

    public static long sleepFromExtakeOutToExtakeIn = 1000, sleepFromExtakeInToIntakeIn = 1000;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized d = new SampleMecanumDriveREVOptimized(hardwareMap);

        ss.init(hardwareMap);
        //s.init();

        waitForStart();

        //TODO reimpl.
        //skyStoneLocation = s.findSkystone();
        skyStoneLocation = Sensors.SkyStoneLocation.LEFT;

        //TODO just make enum value equal to the skystone location so no if loops reqd., check if roadrunner can still work w that
        double skystonePositionX;
        if(skyStoneLocation == Sensors.SkyStoneLocation.LEFT)
            skystonePositionX = skystoneLeftX;
        else if (skyStoneLocation == Sensors.SkyStoneLocation.CENTER)
            skystonePositionX = skystoneCenterX;
        else
            skystonePositionX = skystoneRightX;

        if (isStopRequested()) return;

        //starting at -35, 60
        d.setPoseEstimate(new Pose2d(startingX, startingY, standardHeading));

        //make a straight line strafe in front of skystone
        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .lineTo(new Vector2d(skystoneLeftX, skystoneY), new ConstantInterpolator(standardHeading))
//                        .splineTo(new Pose2d(skystonePositionX, skystoneY, standardHeading))
                        .build() );

        d.turnSync(Math.toRadians(-135));


        // resetting extake so that it doesnt cause the stone to enter sideways
        ss.extakeOut();
        sleep(sleepFromExtakeOutToExtakeIn);
        ss.extakeIn();
        sleep(sleepFromExtakeInToIntakeIn);

        //intake
        ss.intake(0.75); //TODO: ALL STONE SCORER FUNCTIONS NEED TO BE CHANGED

        //TODO i think this is wrong, it needs to move forward and then back (fixed)
        //drives forward to pick up
        d.followTrajectorySync(
                d.trajectoryBuilder()
                        //y used to be 50, too far
                        //.lineTo(new Vector2d(skystonePositionX, 10), new ConstantInterpolator(-90))
                        .forward(distanceForwardToPickUpStone)
                        .build()
        );

        //block picked up
        ss.intake(0);

        //TODO come up with better way to pull out, probably can go backwards, rn will interfere with blocks
        //pull out of area
//        d.followTrajectorySync(
//                d.trajectoryBuilder()
//                        .reverse()
//                        .splineTo(new Pose2d(pulloutX, pulloutY, pulloutHeading))
//                        //.back(distanceForwardToPickUpStone)
//                        .build()
//        );

        telemetry.addData("stat", "after pullout");

        // move over to foundation side
        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .reverse()
                        //.splineTo(new Pose2d(-5, 38, -180), new LinearInterpolator(-90, -180))
                        //.strafeLeft(distanceStrafeLeftForFoundationSide)
                        //.reverse()
                        .lineTo(new Vector2d(skystonePositionX, 33))
                        .build()
        );





        d.turnSync(Math.toRadians(-90-rotationBias));

        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .reverse()
                        .lineTo(new Vector2d(20, 40))
                        .build()
        );

        telemetry.addData("Stat", "after something");
        telemetry.addData("Heading", d.getPoseEstimate().getHeading());
        telemetry.update();

        //d.turnSync(Math.toRadians(180)-d.getPoseEstimate().getHeading());

        sleep(10000);

        //d.getPoseEstimate().getHeading();


        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(30, 35, 90))
                        //.strafeLeft(distanceStrafeLeftForFoundationSide)
                        .build()
        );

        d.turnSync(Math.toRadians(headingForStoneDrop));

        //TODO extake
        ss.extakeOut();
        sleep(100);
        ss.dropStone();
        sleep(100);
        ss.extakeIn();
        sleep(100);

        //park under the bridge
        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .lineTo(new Vector2d(0, pulloutY), new ConstantInterpolator(90))
                        //.strafeLeft(25)
                        .build()
        );











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
}

