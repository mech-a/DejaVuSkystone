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

// CASE A: Next to wall
@Autonomous(name = "Case 3 A Red", group = "Auton")
public class Case3ARed extends LinearOpMode {
    Drivetrain d = new Drivetrain(this);
    StoneScorer ss = new StoneScorer(this);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        d.init();
        ss.init();

        waitForStart();

        if (isStopRequested()) return;

        //starting at 30, -60
        drive.setPoseEstimate(new Pose2d(30, -60, 0));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(30, -30, 0))
                        .splineTo(new Pose2d(50, -30, 0))
                        .build()
        );

        ss.setBlock(10, 10);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(50, -60, 0))
                        .build()
        );

        ss.setBlock(-10, -10);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, -60, 0))
                        .build()
        );
    }
}

