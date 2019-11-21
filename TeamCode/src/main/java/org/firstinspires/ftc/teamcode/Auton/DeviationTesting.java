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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Drivetrain;

import java.util.ArrayList;


/**
 * Copy Me Linear
 */

@TeleOp(name="DeviationTesting", group="Internal")
//@Disabled
public class DeviationTesting extends LinearOpMode {

    // Declare OpMode members.
    Drivetrain d = new Drivetrain(this);

    Drivetrain.Direction dir = Drivetrain.Direction.FWD;

    ArrayList<StringDouble> control = new ArrayList<>();


    @Override
    public void runOpMode() {


        d.init();

        //default values
        control.add(new StringDouble("inches", 15, 0.5));
        control.add(new StringDouble("speed", 0.3, 0.05));

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        int num = 0;
        StringDouble current = control.get(num);

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                current.dub += current.increment;
                sleep(200);
            }

            if (gamepad1.dpad_down) {
                current.dub -= current.increment;
                sleep(200);
            }

            if (gamepad1.dpad_left) {
                if (num == 0) {
                    current = control.get(control.size() - 1);
                } else {
                    current = control.get(num--);
                }
                sleep(200);
            }

            if (gamepad1.dpad_right) {
                if (num == control.size() - 1) {
                    current = control.get(0);
                } else {
                    current = control.get(num++);
                }
                sleep(200);
            }

            telemetry.addData("Inches: ", control.get(0).dub);
            telemetry.addData("Speed: ", "%.2f", control.get(1).dub);
            telemetry.update();

            if (gamepad1.a) {
                d.translate(dir, control.get(0).dub, control.get(1).dub);
            }

        }
    }
}

//TODO make generic tuples utility
class StringDouble {

    String str;
    double dub;
    double increment;

    public StringDouble(String str, double dub, double increment) {
        this.str = str;
        this.dub = dub;
        this.increment = increment;
    }


}
