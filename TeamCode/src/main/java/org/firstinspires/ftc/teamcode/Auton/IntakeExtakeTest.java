package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Drivetrain;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import static org.firstinspires.ftc.teamcode.Auton.Case1A.sleepFromExtakeInToIntakeIn;
import static org.firstinspires.ftc.teamcode.Auton.Case1A.sleepFromExtakeOutToExtakeIn;


@TeleOp(group="test")
@Config
public class IntakeExtakeTest extends LinearOpMode {

    StoneScorer ss = new StoneScorer(this);
    Drivetrain d = new Drivetrain(this);
  //  public static double distanceForwardToPickUpStone = 48;
    public static double pow = 0.5;
    public static double timeBeforeStopping = 2000;



    @Override
    public void runOpMode() {
        //SampleMecanumDriveREVOptimized d = new SampleMecanumDriveREVOptimized(hardwareMap);

        d.init(hardwareMap);

        ss.init(hardwareMap);

        waitForStart();

//        d.followTrajectorySync(
//                d.trajectoryBuilder()
//                .forward(20)
//                .build()
//        );


        //TODO make note to edit constants in case 1a
        ss.extakeOut();
        sleep(sleepFromExtakeOutToExtakeIn);
        ss.extakeIn();
        sleep(sleepFromExtakeInToIntakeIn);

        //intake
        ss.intake(0.75); //TODO: ALL STONE SCORER FUNCTIONS NEED TO BE CHANGED

        d.setPowers(pow,pow,pow,pow);
        sleep((long) (timeBeforeStopping));


        //TODO i think this is wrong, it needs to move forward and then back (fixed)
        //drives forward to pick up
//        d.followTrajectorySync(
//                d.trajectoryBuilder()
//                        //y used to be 50, too far
//                        //.lineTo(new Vector2d(skystonePositionX, 10), new ConstantInterpolator(-90))
//                        .forward(distanceForwardToPickUpStone)
//                        .build()
//        );

        //block picked up
        ss.intake(0);

    }
}
