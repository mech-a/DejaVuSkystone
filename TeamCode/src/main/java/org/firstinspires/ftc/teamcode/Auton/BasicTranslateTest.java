package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Drivetrain;

/**
 *
 */


@Autonomous(name="basic translate test", group="tests")
//@Disabled
public class BasicTranslateTest extends LinearOpMode {
    Drivetrain d = new Drivetrain(this);
    Drivetrain.Direction dir = Drivetrain.Direction.FWD;
    double toMove = 10d;
    double spd = 0.5;

    @Override
    public void runOpMode() {
        d.init();

        waitForStart();

        d.translate(dir, toMove, spd);
    }
}
