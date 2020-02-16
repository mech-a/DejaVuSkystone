package org.firstinspires.ftc.teamcode.Teleop;

import android.security.keystore.StrongBoxUnavailableException;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Drivetrain;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;


@TeleOp(group="Pre-Deploy")
@Config
//@Disabled
public class ThirdCompTeleOp extends LinearOpMode {

    Drivetrain d = new Drivetrain(this);
    StoneScorer ss = new StoneScorer(this);


    @Override
    public void runOpMode() {
        d.init(hardwareMap);
        ss.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {



        }
    }



}
