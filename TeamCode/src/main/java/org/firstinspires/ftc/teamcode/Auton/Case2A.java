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
import org.firstinspires.ftc.teamcode.Assemblies.Sensors;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;


@Autonomous(name = "Case 2 A", group = "Auton")
public class Case2A extends LinearOpMode {
    int leftCaseMovement = 5;

    Drivetrain d = new Drivetrain(this);
    StoneScorer ss = new StoneScorer(this);
    Sensors s = new Sensors(this);
    Sensors.SkyStoneLocation skyStoneLocation;

    @Override
    public void runOpMode() {
        d.init();
        ss.init();
        s.init();

        waitForStart();

        // findSkystone();
        skyStoneLocation = s.findSkystone();


        //perform all operations to find location from this
        if (skyStoneLocation == Sensors.SkyStoneLocation.LEFT) {
            d.translate(Drivetrain.Direction.RIGHT, leftCaseMovement, 0.5);
        } else if (skyStoneLocation == Sensors.SkyStoneLocation.CENTER) {
            d.translate(Drivetrain.Direction.LEFT, -leftCaseMovement + 8, 0.5);
        } else if (skyStoneLocation == Sensors.SkyStoneLocation.RIGHT) {
            d.translate(Drivetrain.Direction.LEFT, -leftCaseMovement + 16, 0.5);
        }
        d.translate(Drivetrain.Direction.FWD, 28, 0.25);

        ss.setBlock(1200,1400);
        d.translate(Drivetrain.Direction.BACK, 10, 0.15);
        ss.liftH(-950);
        d.translate(Drivetrain.Direction.LEFT, 2, 0.15);
        ss.setBlock(1170,1400);
        d.translate(Drivetrain.Direction.BACK, 16, 0.25);
        // 1 means rolled in, is the power set
        // retracted by 1050
        // lift it 900, halfway up
        // retracted by 525
        ss.intake(1,-1800);
        ss.roll2(0);

        d.translate(Drivetrain.Direction.LEFT, 36, 0.25);


        d.translate(Drivetrain.Direction.LEFT, 36, 0.25);

        // can extend up to 725, 700 for now
        ss.extake(1000, -1000, -1, -1800);

        ss.roll2(0);
        d.translate(Drivetrain.Direction.RIGHT, 44, 0.25);
        d.translate(Drivetrain.Direction.FWD, 18,0.25);
        //ss.intake(1, 1);
        d.translate(Drivetrain.Direction.LEFT, 36, 0.25);
        //ss.extake(1,1);

        // move left to be aligned to the foundation
        d.translate(Drivetrain.Direction.LEFT, 30, 0.25);
        //ss.hookFoundation(1);

        // back up to the wall with the foundation
        d.translate(Drivetrain.Direction.BACK, 30, 0.25);
        //ss.hookFoundation(-1);

        // move right out from behind the foundation
        d.translate(Drivetrain.Direction.RIGHT, 30, 0.25);

        // move forward to avoid other robot
        d.translate(Drivetrain.Direction.FWD, 24, 0.25);

        // move right to park under the bridge
        d.translate(Drivetrain.Direction.RIGHT, 18, 0.25);

        s.shutdown();
    }
}
