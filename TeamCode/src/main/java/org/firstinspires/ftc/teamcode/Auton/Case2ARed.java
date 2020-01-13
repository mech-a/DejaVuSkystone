package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Drivetrain;
import org.firstinspires.ftc.teamcode.Assemblies.Sensors;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;

/*
 * This is an example of a more complex path to really test the tuning.
 */

// CASE A: Next to wall
@Autonomous(group = "drive")
public class Case2ARed extends LinearOpMode {
    Drivetrain d = new Drivetrain(this);
    StoneScorer ss = new StoneScorer(this);
    Sensors s = new Sensors(this);

    Sensors.SkyStoneLocation skyStoneLocation;

    @Override
    public void runOpMode() throws InterruptedException {
        d.init();
        ss.init();
        s.init();

        waitForStart();

        skyStoneLocation = s.findSkystone();

        if (isStopRequested()) return;

        //starting at -35, -60
        d.setPoseEstimate(new Pose2d(-35, 60, 0));

        //following needs to be matched to case 1, copied over from case 3
        if (skyStoneLocation == Sensors.SkyStoneLocation.LEFT) {
            d.followTrajectorySync(
                    d.trajectoryBuilder()
                            .splineTo(new Pose2d(-28, 30, 0))
                            .build() );
        } else if (skyStoneLocation == Sensors.SkyStoneLocation.CENTER) {
            d.followTrajectorySync(
                    d.trajectoryBuilder()
                            .splineTo(new Pose2d(-35, 30, 0))
                            .build() );
        } else if (skyStoneLocation == Sensors.SkyStoneLocation.RIGHT) {
            d.followTrajectorySync(
                    d.trajectoryBuilder()
                            .splineTo(new Pose2d(-45, 30, 0))
                            .build() );
        }

        //intake
        ss.setBlock(10, 10); //TODO: ALL STONE SCORER FUNCTIONS NEED TO BE CHANGED

        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .splineTo(new Pose2d(55, 28, -30))
                        .build()
        );

        //extake
        ss.setBlock(-10, -10);
        //foundation
        ss.setBlock(10, 10);

        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .splineTo(new Pose2d(35, 54, 0))
                        .splineTo(new Pose2d (50, 54, 0))
                        .build()
        );

        //lift foundation hooks
        ss.setBlock(10, 10);

        if (skyStoneLocation == Sensors.SkyStoneLocation.LEFT) {
            d.followTrajectorySync(
                    d.trajectoryBuilder()
                            .splineTo(new Pose2d(-52, 30, 0))
                            .build() );
        } else if (skyStoneLocation == Sensors.SkyStoneLocation.CENTER) {
            d.followTrajectorySync(
                    d.trajectoryBuilder()
                            .splineTo(new Pose2d(-60, 30, 0))
                            .build() );
        } else if (skyStoneLocation == Sensors.SkyStoneLocation.RIGHT) {
            d.followTrajectorySync(
                    d.trajectoryBuilder()
                            .splineTo(new Pose2d(-68, 30, 0))
                            .build() );
        }

        //intake
        ss.setBlock(10, 10); //TODO: ALL STONE SCORER FUNCTIONS NEED TO BE CHANGED

        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .splineTo(new Pose2d(50, 54, 0))
                        .build()
        );

        //extake
        ss.setBlock(-10, -10);

        //parking next to wall
        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .splineTo(new Pose2d(0, 60, 0))
                        .build()
        );
    }
}

