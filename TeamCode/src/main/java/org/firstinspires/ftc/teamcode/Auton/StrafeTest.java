package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;


// CASE A: Next to wall
@Config
@Autonomous(group = "drive")
public class StrafeTest extends LinearOpMode {
    public static double standardHeading = 0;
    public static double startingX = 0, startingY = 0;
    public static double goalX = 24, goalY = 24;
    public static double angle = 45;



    //instruct 48, 36; 36, 26.5; 60, 48


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized d = new SampleMecanumDriveREVOptimized(hardwareMap);

        waitForStart();

        //starting at -35, 60
        d.setPoseEstimate(new Pose2d(startingX, startingY, standardHeading));

        //make a straight line strafe in front of skystone
        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .lineTo(new Vector2d(goalX, goalY), new LinearInterpolator(Math.toRadians(standardHeading), Math.toRadians(angle))
                        )
//                        .splineTo(new Pose2d(skystonePositionX, skystoneY, standardHeading))
                        .build());

    }

    public static double strafeConvert(double distance) {
        return (1.2 * distance + 3.53);
    }
}



