package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.path.*;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

public class TrajectoryTesting extends LinearOpMode {
    StoneScorer ss = new StoneScorer(this);

    public static double foundationRightX = -81;
    public static double foundationRightY = 9;

    public static double pullfoundationRightX = 12;
    public static double pullfoundationRightY = 28;
    public static double pullfoundationHeading = 90;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDriveREVOptimized d = new SampleMecanumDriveREVOptimized(hardwareMap);

        d.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();

        //strafe right and cross under bridge
        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(foundationRightX, foundationRightY, Math.toRadians(-90)))
                        .build()
        );

        LineSegment line = new LineSegment(
                new Vector2d(0, 0),
                new Vector2d( -6, 0)
        );

        PathSegment segment = new PathSegment(line);

        Path path = new Path(segment);

        //DriveConstraints

        //Trajectory trajectory = TrajectoryGenerator.generateTrajectory(path, d.getCon)

        //d.followTrajectorySync(trajectory);

    }
}
