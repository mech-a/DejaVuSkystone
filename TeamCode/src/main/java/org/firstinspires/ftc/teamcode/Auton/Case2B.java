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

/**
 * Assumes that alliance partner does not foundation, either parks or doesn't park.
 * Robot moves skystones, then hooks foundation to back up and move it into the building site.
 * Unhooks from foundation, then parks away from wall.
 */

@Autonomous(name = "Case 2 B", group = "Auton")
public class Case2B extends LinearOpMode {
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


        //based on enum return from CV, translates accordingly to relative position of skystone from starting position
        if (skyStoneLocation == Sensors.SkyStoneLocation.LEFT) {
            d.translate(Drivetrain.Direction.RIGHT, leftCaseMovement, 0.5);
        } else if (skyStoneLocation == Sensors.SkyStoneLocation.CENTER) {
            d.translate(Drivetrain.Direction.RIGHT, -leftCaseMovement + 8, 0.5);
        } else if (skyStoneLocation == Sensors.SkyStoneLocation.RIGHT) {
            d.translate(Drivetrain.Direction.RIGHT, -leftCaseMovement + 16, 0.5);
        }

        //approach skystone
        d.translate(Drivetrain.Direction.FWD, 26, 0.25);

        //extend and lower intake mechanism, pull skystone in
        ss.setBlock(1285,1810);
        d.translate(Drivetrain.Direction.BACK, 10, 0.15);
        //intake mechanism raised, robot translate, mechanism extended and lowered again to keep block in position
        ss.liftH(-1200);
        d.translate(Drivetrain.Direction.RIGHT, 2, 0.15);
        ss.setBlock(1170,1400);
        d.translate(Drivetrain.Direction.BACK, 6, 0.25);
        // 1 means rolled in, is the power set
        // retracted by 1050
        // lift it 900, halfway up
        // retracted by 525
        ss.intake(1,-1800);
        ss.roll2(0);

        d.translate(Drivetrain.Direction.LEFT, 36, 0.25);

        // can extend up to 725, but keeping the value as a safe 700 for now
        ss.extake(1000, -1000, -1, -1800);

        ss.roll2(0);
        d.translate(Drivetrain.Direction.RIGHT, 44, 0.25);
        d.translate(Drivetrain.Direction.FWD, 18,0.25);
        //ss.intake(1, 1);
        d.translate(Drivetrain.Direction.LEFT, 36, 0.25);
        //ss.extake(1,1);

        // translate left to be aligned to the foundation
        d.translate(Drivetrain.Direction.LEFT, 30, 0.25);
        //ss.hookFoundation(1);

        // back up to the wall with the foundation, pulling it into building site
        d.translate(Drivetrain.Direction.BACK, 30, 0.25);
        //ss.hookFoundation(-1);

        // translate right out from behind the foundation
        d.translate(Drivetrain.Direction.RIGHT, 30, 0.25);

        // translate forward to avoid parked alliance partner
        d.translate(Drivetrain.Direction.FWD, 24, 0.25);

        // translate right to park under the bridge
        d.translate(Drivetrain.Direction.RIGHT, 18, 0.25);

        s.shutdown();
    }
}
