/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Assemblies.Drivetrain;


/**
 * Second Competition Tele-op
 * Made by Gaurav
 */


//ftc dashboard implementation: https://acmerobotics.github.io/ftc-dashboard/basics
// add static, non-final fields to the class annotated w @Config
@Config
@TeleOp(name="Second Comp Teleop", group="Precomp")
//@Disabled
public class SecondCompTeleOp extends LinearOpMode {
    //Objects
    public enum DriveMode {FIELD, CARTESIAN}

    DriveMode driveMode = DriveMode.FIELD;

    //TODO finish developing stonescorer and sensors
    Drivetrain d = new Drivetrain(this);

    //left joystick x, y; right joystick x, y
    double[] g1Joy = new double[4];
    double[] g2Joy = new double[4];

    //Fields
    //TODO move to config/constants, keeping in for ftcdashboard
    public static double slowMod = 0.05, fastMod = 0.325;
    boolean runFast = true, runSlow = false;

    public static double joystickDeadzone = 0.1;

    double powFL, powFR, powBR, powBL;
    double[] dtPowers = {powFL, powFR, powBR, powBL};





    //Dependent on fields
    double[] speedSwitch = {slowMod, fastMod};
    double modifier = speedSwitch[1];





    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        d.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            joystickArraysRefresh();


            applyModifiers();
        }
    }


    private void joystickArraysRefresh() {
        //TODO look at Gamepad.cleanMotionValues for scaling joystick vals after deadzone & clipping
        Gamepad[] gamepads = {gamepad1, gamepad2};
        double[][] joysticks = {g1Joy, g2Joy};

        for(int i = 0; i<gamepads.length; i++) {
            //TODO consider moving to byte[] (see Gamepad.toByteArray) for redundancy reduction
            joysticks[i][0] = joystickAxisNormalization(gamepads[i].left_stick_x);
            joysticks[i][1] = -joystickAxisNormalization(gamepads[i].left_stick_y);
            joysticks[i][2] = joystickAxisNormalization(gamepads[i].right_stick_x);
            joysticks[i][3] = -joystickAxisNormalization(gamepads[i].right_stick_y);
        }
    }

    //TODO lambda functions/functional paradigms through kotlin to clean this up
    private double joystickAxisNormalization(double axisValue) {
        //currently only deadzoning
        return (Math.abs(axisValue) > joystickDeadzone ? axisValue : 0);
    }

    private void applyModifiers() {
        for(int i = 0; i<dtPowers.length; i++)
            dtPowers[i]*=modifier;
    }
}
