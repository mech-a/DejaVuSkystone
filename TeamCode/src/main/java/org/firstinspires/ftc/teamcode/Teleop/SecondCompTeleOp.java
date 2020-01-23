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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

    //////////////////////////////////////////////////////////////
    DcMotor leftRoller, rightRoller;
    Servo clawServo, ferrisServo, rotationServo, foundationServo;

    //TODO enumerate
    int extake_position = 0; ////-1 for in and 1 for out
    //TODO figure out
    int intake = 0;
    boolean release = true;
    ElapsedTime clawTimer = new ElapsedTime();

    //imu info
    private BNO055IMU imu;
    private BNO055IMU.Parameters gyroParameters;
    private double heading;
    private Orientation angles;

    //tunables
    static double
            init_rotationServo = 0.66, min_rotationServo = 0, max_rotationServo = 0.66, step_rotationServo = 0.015,
            init_ferrisServo = 0.32, x_ferrisServo = 0.6989, min_ferrisServo = 0.32, max_ferrisServo = 0.86, step_ferrisServo = 0.005,
            init_clawServo = 1, min_clawServo = 0.62, max_clawServo = 1, dpadLeft_clawServo = 0.862,
            init_foundationServo = 0.8, step_foundationServo = 0.01;



    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        d.init();
        //TODO stonescorer
        initServos();
        //TODO understand why we hang at imuinit, logs dl'd on g-laptop
        imuInit();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            joystickArraysRefresh();

            //TODO consolidate into StoneScorer, Sensors, and Constants
            //x - extend extake, y - bring in extake
            if(gamepad2.x) {
                leftRoller.setPower(0);
                rightRoller.setPower(0);
                intake = 0;

                clawTimer.reset();
                clawServo.setPosition(0.62);
            }
            else if(gamepad2.y) {
                clawTimer.reset();
                extake_position = -1;
                clawServo.setPosition(1);
                ferrisServo.setPosition(0.6989);
                rotationServo.setPosition(0.66);
            }
            else if (gamepad2.a) {
                extake_position = 1;
            }

            if(extake_position == 1 && clawTimer.milliseconds() > 500) {
                ferrisServo.setPosition(0.86);      //ferris servo has limits 0.577 and 0.0522
                rotationServo.setPosition(0.0);  //rotation servo has limits 0.03 and 0.54
                extake_position = 0;
            }
            if(extake_position == -1 && clawTimer.milliseconds() > 300) {
                ferrisServo.setPosition(0.32);
                extake_position = 0;
            }

            // temporary servo control
            if(gamepad2.dpad_up) {
                rotationServo.setPosition(rotationServo.getPosition() + 0.015);
                //clawServo.setPosition(clawServo.getPosition() + 0.005);
            }
            if(gamepad2.dpad_down) {
                rotationServo.setPosition(rotationServo.getPosition() - 0.015);
                //clawServo.setPosition(clawServo.getPosition() - 0.005);
            }
            if(gamepad2.dpad_right) {
                clawServo.setPosition(0.62);
            }
            if(gamepad2.dpad_left) {
                clawServo.setPosition(0.862);
            }
            if(gamepad2.a) {
                ferrisServo.setPosition(ferrisServo.getPosition() + 0.005);
                //clawServo.setPosition(clawServo.getPosition() + 0.005);
            }
            if(gamepad2.b) {
                ferrisServo.setPosition(ferrisServo.getPosition() - 0.005);
                //clawServo.setPosition(clawServo.getPosition() - 0.005);
            }




            applyModifiers();

            setPowers();
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

    private void setPowers(){
        d.setPowers(powFL, powFR, powBR, powBL);

    }

    private void imuInit() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyroParameters.loggingEnabled      = true;
        gyroParameters.loggingTag          = "IMU";

        // Default is 32
        // TODO check powerdrain
        gyroParameters.gyroBandwidth = BNO055IMU.GyroBandwidth.HZ523;

        imu.initialize(gyroParameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }


    public void initServos() {
        // in order of configuration
        rotationServo = hardwareMap.get(Servo.class, "rotation_servo");
        ferrisServo = hardwareMap.get(Servo.class, "ferris_servo");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        foundationServo = hardwareMap.get(Servo.class, "foundation_servo");

        // initialization points for servos
        rotationServo.setPosition(init_rotationServo);
        ferrisServo.setPosition(init_ferrisServo);
        clawServo.setPosition(init_clawServo);
        foundationServo.setPosition(0.8);

        rotationServo.setDirection(Servo.Direction.FORWARD);
        ferrisServo.setDirection(Servo.Direction.FORWARD);
        foundationServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);
    }




}
