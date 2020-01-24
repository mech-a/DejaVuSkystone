package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Assemblies.ConfigurationData;
import org.firstinspires.ftc.teamcode.Assemblies.RRMergedDrivetrain;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

/*
 * This is an example of a more complex path to really test the tuning.
 */

// CASE A: Next to wall
@Autonomous(name = "Case 3 A Blue", group = "Auton")
public class Case3A extends LinearOpMode {
    StoneScorer ss = new StoneScorer(this);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREV d = new SampleMecanumDriveREV(hardwareMap);
        ss.init();

        Servo foundationServo;
        foundationServo = hardwareMap.get(Servo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[3]);
        foundationServo.setPosition(0.65);
        foundationServo.setDirection(Servo.Direction.FORWARD);


        waitForStart();

        if (isStopRequested()) return;

        //starting at 30, 60
        d.setPoseEstimate(new Pose2d(30, 60, 90));

        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .lineTo(new Vector2d(30, 35), new ConstantInterpolator(90))
                        .lineTo(new Vector2d(60, 30), new ConstantInterpolator(90))
                        .build()
        );

        sleep(500);
        //foundation
        foundationServo.setPosition(0);

        sleep(1000);


        d.followTrajectorySync(
                d.trajectoryBuilder()
                        .lineTo(new Vector2d(30, 42), new ConstantInterpolator(90))
                        .splineTo(new Pose2d(30, 52, 180)//, new ConstantInterpolator(180)
                        )
                        .lineTo(new Vector2d(50, 52), new ConstantInterpolator(180))
                        .build()
        );
    }}

/*

        //lift up foundation hooks
        ss.setBlock(-10, -10);

        d.followTrajectorySync(
                d.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 60, 0))
                    .build()
        );
    }
}
*/
