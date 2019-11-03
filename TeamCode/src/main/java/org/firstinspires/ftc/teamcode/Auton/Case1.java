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


@Autonomous(name = "Case 1", group = "Auton")
public class Case1 extends LinearOpMode {
    int liftValueA = -5400;
    int extendValueA = 2260;

    Drivetrain d = new Drivetrain(this);
    StoneScorer ss = new StoneScorer(this);

    @Override
    public void runOpMode() {
        d.init();
        ss.init();

        waitForStart();

        // findSkystone();

        // obtain skystone
        ss.setBlock(-1000, 1800, 1000);
        ss.setBlock(0, 0,0);
        d.translate(Drivetrain.Direction.BACK, 12, 0.25);

        d.translate(Drivetrain.Direction.BACK, 12, 0.25);
        ss.intake(1, 1);

        d.translate(Drivetrain.Direction.LEFT, 36, 0.25);
        ss.extake(1, extendValueA, -1,1);

        // move right to position robot in front of second skystone
        d.translate(Drivetrain.Direction.RIGHT, 36, 0.25);

        // obtain skystone
        ss.setBlock(liftValueA, extendValueA, 0);
        d.translate(Drivetrain.Direction.BACK, 12, 0.25);
        ss.intake(1, 1);

        // move left all the way into build zone
        d.translate(Drivetrain.Direction.LEFT, 36, 0.25);
        ss.extake(1, extendValueA, -1,1);

        // move right to park under bridge
        d.translate(Drivetrain.Direction.RIGHT, 36, 0.25);
    }
}
