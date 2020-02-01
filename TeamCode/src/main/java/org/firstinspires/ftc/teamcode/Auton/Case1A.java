package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.RRMergedDrivetrain;
import org.firstinspires.ftc.teamcode.Assemblies.Sensors;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.yaml.snakeyaml.scanner.Constant;

import java.util.Vector;

/*
 * This is an example of a more complex path to really test the tuning.
 */

// CASE A: Next to wall
@Autonomous(group = "drive")
public class Case1A extends LinearOpMode {
    StoneScorer ss = new StoneScorer(this);
    //Sensors s = new Sensors(this);

    Sensors.SkyStoneLocation skyStoneLocation;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREV d = new SampleMecanumDriveREV(hardwareMap);

        ss.init();
        //s.init();

        waitForStart();

        //TODO reimpl.
        //skyStoneLocation = s.findSkystone();
        skyStoneLocation = Sensors.SkyStoneLocation.LEFT;

        double skystonePositionX;
        if(skyStoneLocation == Sensors.SkyStoneLocation.LEFT) {
            skystonePositionX = -28;
        }
        else if (skyStoneLocation == Sensors.SkyStoneLocation.CENTER) {
            skystonePositionX = -35;
        }
        else {
            skystonePositionX = -45;
        }

        if (isStopRequested()) return;

        //starting at -35, 60
        d.setPoseEstimate(new Pose2d(-35, 60, -90));

        //following needs to be matched to case 1, copied over from case 3
        if (skyStoneLocation == Sensors.SkyStoneLocation.LEFT) {
            d.followTrajectorySync(
                    d.trajectoryBuilder()
                            .lineTo(new Vector2d(skystonePositionX, 30), new ConstantInterpolator(-90))
                            .build() );
        } else if (skyStoneLocation == Sensors.SkyStoneLocation.CENTER) {
            d.followTrajectorySync(
                    d.trajectoryBuilder()
                            //.splineTo(new Pose2d(-35, -30, 0))
                            //TODO check odd behavior for center
                            //TODO make vars.
                            .lineTo(new Vector2d(skystonePositionX, 30), new ConstantInterpolator(-90))
                            .build() );
        } else if (skyStoneLocation == Sensors.SkyStoneLocation.RIGHT) {
            d.followTrajectorySync(
                    d.trajectoryBuilder()
                           // .splineTo(new Pose2d(-45, -30, 0))
                            .lineTo(new Vector2d(skystonePositionX, 30), new ConstantInterpolator(-90))
                            .build() );
        }

        //intake
        ss.intake(0.75); //TODO: ALL STONE SCORER FUNCTIONS NEED TO BE CHANGED

        //TODO i think this is wrong, it needs to move forward and then back (fixed)
        //drives forward to pick up
        d.followTrajectorySync(
                d.trajectoryBuilder()
                        //y used to be 50, too far
                        .lineTo(new Vector2d(skystonePositionX, 10), new ConstantInterpolator(-90))
                        .build()
        );

        //block picked up
        ss.intake(0);

        //pull out of area
        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .splineTo(new Pose2d(-30, 35, -90))
                        .build()
        );

        // move over to foundation side
        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .lineTo(new Vector2d(25, 35), new ConstantInterpolator(-90))
                        .build()
        );

        d.turnSync(Math.toRadians(90));

        //TODO extake
        ss.extakeOut();
        sleep(100);
        ss.dropStone();
        sleep(100);
        ss.extakeIn();
        sleep(100);

        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .lineTo(new Vector2d(0, 35), new ConstantInterpolator(90))
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

