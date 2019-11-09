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

package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Drivetrain;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;


@Autonomous(name = "Test Case", group = "Auton")
public class Testing extends LinearOpMode {
    int liftValueA = -5400;
    int extendValueA = 2260;
    int i = 1;

    Drivetrain d = new Drivetrain(this);
    StoneScorer ss = new StoneScorer(this);

    @Override
    public void runOpMode() {
        d.init();
        ss.init();

        waitForStart();

        d.translate(Drivetrain.Direction.FWD, 24, 0.25);
        // first
        if (i == 1) {
            d.translate(Drivetrain.Direction.LEFT, 8, 0.25);
        } else if (i == 2) {
            d.translate(Drivetrain.Direction.RIGHT, 4, 0.25);
        } else if (i == 3) {
            d.translate(Drivetrain.Direction.RIGHT, 12, 0.25);
        }

        // getting block
        d.translate(Drivetrain.Direction.BACK, 10, 0.15);
        d.translate(Drivetrain.Direction.LEFT, 2, 0.15);
        d.translate(Drivetrain.Direction.BACK, 6, 0.25);

        // translate over to other side
        d.translate(Drivetrain.Direction.LEFT, 40, 0.25);
        d.translate(Drivetrain.Direction.RIGHT, 64, 0.25);
        d.translate(Drivetrain.Direction.FWD, 18,0.25);

        // getting block
        d.translate(Drivetrain.Direction.BACK, 10, 0.15);
        d.translate(Drivetrain.Direction.LEFT, 2, 0.15);
        d.translate(Drivetrain.Direction.BACK, 6, 0.25);

        //ss.intake(1, 1);
        d.translate(Drivetrain.Direction.LEFT, 60, 0.25);
        //ss.extake(1,1);

        // translate left to align with foundation, 4th nub
        d.translate(Drivetrain.Direction.LEFT, 20, 0.25);
        d.translate(Drivetrain.Direction.FWD, 2, 0.25);

        // hook onto foundation
        //ss.hookFoundation(1, 2400);
        //ss.extendH(-300);

        // drag foundation back
        d.translate(Drivetrain.Direction.BACK, 27, 0.25);


        // unhook the foundation
        //ss.hookFoundation(0, 2200);

        // CASE A: translate right park to under bridge
        d.translate(Drivetrain.Direction.RIGHT, 27, 0.25);
        d.translate(Drivetrain.Direction.FWD, 24, 0.25);
        d.translate(Drivetrain.Direction.RIGHT, 27, 0.25);

        //d.translate(Drivetrain.Direction.RIGHT, 54, 0.25);


    }
}
