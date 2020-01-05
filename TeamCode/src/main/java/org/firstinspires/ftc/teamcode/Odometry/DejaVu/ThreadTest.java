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

package org.firstinspires.ftc.teamcode.Odometry.DejaVu;

import android.provider.Settings;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Drivetrain;

import java.util.ArrayList;


/**
 * Copy Me Linear
 */

@TeleOp(name="ThreadTest", group="Internal")
//@Disabled
public class ThreadTest extends LinearOpMode {

    // Declare OpMode members.
    Drivetrain d = new Drivetrain(this);


    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)

        d.init();

        GlobalPosition gp = new GlobalPosition(d, 75);
        Thread gpThread = new Thread(gp);

        gpThread.start();

        telemetry.addData("Stat", "ready");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)

            telemetry.addData("position 0", gp.getPositionArray().get(0));
            telemetry.addData("position 1", gp.getPositionArray().get(1));
            telemetry.update();
            driveTo(0, 1000, 20);

        gp.shutdown();

    }

    public void driveTo(double y, double x, double maxError) {
//        //inputs: error function, PID coeffs: outputs: motor powers
        ArrayList<Double> position = new ArrayList<>();

        double pX;
        double dX;
        double iX;
        double outputScalar;

        double kD = 0;
        double kI = 0;
        double kP = 0.002;

        boolean threadEnabled = true;

        for (int i = 0; i < d.getCurrentEncoderValues().size(); i++) {
            position.add(d.getCurrentEncoderValues().get(i));
        }

        while (threadEnabled) {

            double errorX = x - d.getCurrentEncoderValues().get(0);

            if (errorX <= maxError) {
                break;
            }

            pX = kP * errorX;
            //dX = kD * (errorX-lastErrorX)/(iterationTime/1000.0);
            //iX +=kI * (errorX*iterationTime/1000);
            //outputScalar = pX + dX + iX;
            outputScalar = pX;


            telemetry.addData("output scalar", outputScalar);
            telemetry.addData("position FL", d.getCurrentEncoderValues().get(0));
            telemetry.update();
            //Set Motor values to this scalar
            d.mtrFL.setPower(outputScalar);
            d.mtrFR.setPower(outputScalar);
            d.mtrBL.setPower(outputScalar);
            d.mtrBR.setPower(outputScalar);

        }
    }
}

