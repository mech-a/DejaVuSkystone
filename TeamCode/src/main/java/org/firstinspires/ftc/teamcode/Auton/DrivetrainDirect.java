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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Assemblies.Drivetrain;


/**
 * Directly powers the drive wheels with a constant power.
 * Cycle between RUN_USING_ENCODER and RUN_WITHOUT_ENCODER for more testing.
 */

@Autonomous(group="Internal")
//@Disabled
@Config
public class DrivetrainDirect extends LinearOpMode {

    // Declare OpMode members.
    Drivetrain d = new Drivetrain(this);
    public static double power = 0.5, sleep = 500, numRuns = 4;
    public static boolean zeroPowerAfter = false;
    public static boolean forwardBackward = true;

    private DcMotor fl, fr, br, bl;


    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        d.init(hardwareMap);



        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (forwardBackward) {
                //straight
                d.setPowers(power, power, power, power);
                sleep((long) sleep);
                if (zeroPowerAfter)
                    d.setPowers(0, 0, 0, 0);

                //back
                d.setPowers(-power, -power, -power, -power);
                sleep((long) sleep);
                if (zeroPowerAfter)
                    d.setPowers(0, 0, 0, 0);
            } else {
                //right
                d.setPowers(power, -power, power, -power);
                sleep((long) sleep);
                if(zeroPowerAfter)
                    d.setPowers(0,0,0,0);

                //left
                d.setPowers(-power, power, -power, power);
                sleep((long) sleep);
                if(zeroPowerAfter)
                    d.setPowers(0,0,0,0);
            }
        }
    }
}
