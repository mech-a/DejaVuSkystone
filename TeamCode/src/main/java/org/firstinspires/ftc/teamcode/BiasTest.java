package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@TeleOp(group = "drive")
public class BiasTest extends LinearOpMode {
    public static double ANGLE = 90; // deg
    public static double BIAS = 10;
    public static int counter1 = 0;
    int counter0 = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;


        while(opModeIsActive()) {
            if(counter1 == counter0) {
                drive.turnSync(Math.toRadians(ANGLE+BIAS));
                counter0++;
            }
        }
    }
}
