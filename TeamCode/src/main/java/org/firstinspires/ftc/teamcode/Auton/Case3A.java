package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.RRMergedDrivetrain;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;

/*
 * This is an example of a more complex path to really test the tuning.
 */

// CASE A: Next to wall
@Autonomous(name = "Case 3 A Blue", group = "Auton")
public class Case3A extends LinearOpMode {
    RRMergedDrivetrain d = new RRMergedDrivetrain(this);
    StoneScorer ss = new StoneScorer(this);

    @Override
    public void runOpMode() throws InterruptedException {
        d.init();
        ss.init();

        waitForStart();

        if (isStopRequested()) return;

        //starting at 30, 60
        d.setPoseEstimate(new Pose2d(30, 60, 0));

        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .splineTo(new Pose2d(30, 30, 0))
                        .splineTo(new Pose2d(50, 30, 0))
                        .build()
        );

        //foundation
        ss.setBlock(10, 10);

        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .splineTo(new Pose2d(35, 54, 0))
                        .splineTo(new Pose2d (50, 54, 0))
                        .build()
        );

        //lift up foundation hooks
        ss.setBlock(-10, -10);

        d.followTrajectorySync(
                d.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 60, 0))
                    .build()
        );
    }
}

