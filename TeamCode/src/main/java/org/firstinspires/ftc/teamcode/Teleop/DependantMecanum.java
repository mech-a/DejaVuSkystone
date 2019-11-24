package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Drivetrain;
import org.firstinspires.ftc.teamcode.Assemblies.Sensors;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;

/**
 * Placeholder for a Subassembly-dependant Mecanum OpMode
 * Created by Gaurav 11-14-19
 *
 */

@TeleOp(name="Dependent Mecanum", group="Pre-Comp")
public class DependantMecanum extends LinearOpMode {
    public StoneScorer ss;
    public Drivetrain d;
    public Sensors s;

    @Override
    public void runOpMode() {

        waitForStart();

        while(opModeIsActive()) {

        }
    }


    public fieldCentric() {
        double y = -gamepad1.left_stick_y, x = gamepad1.left_stick_x, rot = gamepad1.right_stick_x;

        double joystickAngle = Math.atan2(y, x);
        double magnitude = Math.hypot(x, y);

        double angleToRun = joystickAngle-getHeading();

        double lateralFLBR = magnitude*Math.sin(angleToRun+0.25*Math.PI);
        double lateralFRBL = magnitude*Math.sin(angleToRun-0.25*Math.PI);

        double powFL = (lateralFLBR + rot)/Math.max(lateralFLBR, rot);
        double powFR = (lateralFRBL + rot)/Math.max(lateralFRBL, rot);
        double powBL = (lateralFRBL + rot)/Math.max(lateralFRBL, rot);
        double powBR = (lateralFLBR + rot)/Math.max(lateralFLBR, rot);


    }


}

// I was thinking we could organize all of our subassemblies under another class, but I think it's unnecessary for now.
//class SubassemblyStack {
//    public StoneScorer ss;
//
//}
