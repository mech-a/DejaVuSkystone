package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Drivetrain;
import org.firstinspires.ftc.teamcode.Assemblies.Sensors;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

/*
 * This is an example of a more complex path to really test the tuning.
 */

// CASE B: Away from wall
@Autonomous(name = "Case 1 B Red", group = "drive")
public class Case1BRed extends LinearOpMode {
    //Drivetrain d = new Drivetrain(this);
    StoneScorer ss = new StoneScorer(this);
    Sensors s = new Sensors(this);

    Sensors.SkyStoneLocation skyStoneLocation;

    @Override
    public void runOpMode() throws InterruptedException {
     //   d.init();
        ss.init();
        s.init();

        SampleMecanumDriveBase d = new SampleMecanumDriveREV(hardwareMap);

        waitForStart();

        skyStoneLocation = s.findSkystone();

        if (isStopRequested()) return;

        //starting at -35, 60
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
        ss.intake(0.75);
        sleep(1500);
        ss.intake(0);

        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .splineTo(new Pose2d(20, 50, 0))
                        .build()
        );

        //extake
        ss.extakeOut();
        ss.extakeIn();

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
        ss.intake(0.75);
        sleep(1500);
        ss.intake(0);

        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .splineTo(new Pose2d(20, 50, 0))
                        .build()
        );

        //extake
        ss.extakeOut();
        ss.extakeIn();

        //parking away from wall
        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .splineTo(new Pose2d(0, 37, 0))
                        .build()
        );
    }
}

