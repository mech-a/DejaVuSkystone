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

import org.firstinspires.ftc.teamcode.Assemblies.Drivetrain;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;

// CASE B: Away from wall
@Autonomous(name = "Case 3 B", group = "Auton")
public class Case3B extends LinearOpMode {

    Drivetrain d = new Drivetrain(this);
    StoneScorer ss = new StoneScorer(this);

    @Override
    public void runOpMode() {
        // initialize drivetrain and stone scoring subassemblies
        d.init();
        ss.init();

        waitForStart();

        // move forward up to the foundation
        d.translate(Drivetrain.Direction.FWD, 24, 0.25);
        // translate left to align with foundation, 4th nub
        d.translate(Drivetrain.Direction.LEFT, 20, 0.25);

        // hook onto foundation
        ss.hookFoundation(1, 2300);

        // drag foundation back
        d.translate(Drivetrain.Direction.BACK, 26, 0.25);

        // unhook the foundation
        ss.hookFoundation(0, 2200);

        // translate right to be out of the way of the foundation
        d.translate(Drivetrain.Direction.RIGHT, 32, 0.25);
        // CASE B: move forward to be away from wall
        d.translate(Drivetrain.Direction.FWD, 21, 0.25);
        // move right to park
        d.translate(Drivetrain.Direction.RIGHT, 21, 0.25);
    }
}
