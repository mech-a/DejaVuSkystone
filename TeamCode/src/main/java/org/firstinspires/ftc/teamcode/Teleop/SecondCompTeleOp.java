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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Assemblies.Drivetrain;

import java.util.ArrayList;


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
    public enum TelemetryOutputType {SHORT, DEBUG}

    DriveMode driveMode = DriveMode.FIELD;
    TelemetryOutputType telemetryOutputType = TelemetryOutputType.SHORT;

    //TODO finish developing stonescorer and sensors
    Drivetrain d = new Drivetrain(this);

    //left joystick x, y; right joystick x, y
    //ArrayList<Double> g1Joy = new ArrayList<>();
    //ArrayList<Double> g2Joy = new ArrayList<>();
    double[] g1Joy = new double[4];
    double[] g2Joy = new double[4];

    //Fields
    //TODO move to config/constants, keeping in for ftcdashboard
    public static double slowMod = 0.05, fastMod = 0.325;
    //de-capitalize, not final
    public static double TRIGGER_DEADZONE = 0.1;
    boolean runFast = true, runSlow = false;

    public static double joystickDeadzone = 0.1;

    double powFL, powFR, powBR, powBL;
    double[] dtPowers = {powFL, powFR, powBR, powBL};





    //Dependent on fields
    double[] speedSwitch = {slowMod, fastMod};
    double modifier = speedSwitch[1];

    //////////////////////////////////////////////////////////////
    DcMotor leftRoller, rightRoller, mtrVertical;
    DcMotor mtrFL, mtrFR, mtrBR, mtrBL;
    Servo clawServo, ferrisServo, rotationServo, foundationServo;

    double powMtrVertical;

    private double fwd, rotate, strafe;
    
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
//    static double
//            init_rotationServo = 0.66, min_rotationServo = 0, max_rotationServo = 0.66, step_rotationServo = 0.015,
//            init_ferrisServo = 0.32, x_ferrisServo = 0.6989, min_ferrisServo = 0.32, max_ferrisServo = 0.86, step_ferrisServo = 0.005,
//            init_clawServo = 1, min_clawServo = 0.62, max_clawServo = 1, dpadLeft_clawServo = 0.862,
//            init_foundationServo = 0.8, step_foundationServo = 0.01;
    static double
            clamp_clawServo = 0.643, unclamp_clawServo = 1,
            kickback_ferrisServo = 0.92, place_ferrisServo = 0.85, intake_ferrisServo = 0.31, step_ferrisServo = 0.005,
            up_foundationServo = 0.65, down_foundationServo = 0,
            front_rotationServo = 0.643, back_rotationServo = 0.02, step_rotationServo = 0.015,
            intakeSpeed = 0.75, vertMod = 1.0/3, vertSlideBottomBias = 50, vertSlideMax = 5000, vertSlideTopBias = 50,
    //TODO remember, these are copy by value, don't know how they work with dashboard
            init_rotationServo = front_rotationServo, init_clawServo = unclamp_clawServo, init_foundationServo = up_foundationServo, init_ferrisServo = intake_ferrisServo;

    double rotationServoBias = 28.8/270;








    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //d.init();
        initMotors();
        //TODO stonescorer
        initServos();
        //TODO understand why we hang at imuinit, logs dl'd on g-laptop
        imuInit();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            speedSwitch();
            joystickArraysRefresh();

            //TODO consolidate into StoneScorer, Sensors, and Constants
            //x - extend extake, y - bring in extake
            if(gamepad2.x) {
                //zero out intake
                leftRoller.setPower(0);
                rightRoller.setPower(0);
                intake = 0;

                //clamp onto block clamp_clawServo = 0.62
                clawTimer.reset();
                clawServo.setPosition(clamp_clawServo);
            }
            else if(gamepad2.y) {
                clawTimer.reset();
                extake_position = -1;

                //unclamp servo unclamp_clawServo = 1
                clawServo.setPosition(unclamp_clawServo);
                //kickback ferris servo kickback_ferrisServo = 0.6989
                ferrisServo.setPosition(kickback_ferrisServo);
                //back to default accepting, front_rotationServo = 0.66
                rotationServo.setPosition(front_rotationServo+rotationServoBias);
            }
            else if (gamepad2.a) {
                extake_position = 1;
            }

            if(extake_position == 1 && clawTimer.milliseconds() > 500) {
                //ready to drop position place_ferrisServo = 0.86
                ferrisServo.setPosition(place_ferrisServo);
                //move whole apparatus around to the back back_rotationServo = 0.0
                rotationServo.setPosition(back_rotationServo+rotationServoBias);
                extake_position = 0;
            }
            if(extake_position == -1 && clawTimer.milliseconds() > 300) {
                //ready to take in block intake_ferrisServo = 0.32
                ferrisServo.setPosition(intake_ferrisServo);
                extake_position = 0;
            }

            if(gamepad2.b) {
                clawServo.setPosition(unclamp_clawServo);
            }

            // temporary servo control
            if(gamepad2.dpad_up) {
                //step_rotationServo = 0.015
                rotationServo.setPosition(rotationServo.getPosition() + step_rotationServo);
                //clawServo.setPosition(clawServo.getPosition() + 0.005);
            }
            if(gamepad2.dpad_down) {
                rotationServo.setPosition(rotationServo.getPosition() - step_rotationServo);
                //clawServo.setPosition(clawServo.getPosition() - 0.005);
            }
            if(gamepad2.dpad_right) {
                //close clawServo, change to clamp_clawServo
                clawServo.setPosition(clamp_clawServo);
            }
            if(gamepad2.dpad_left) {
                //open clawServo, change to unclamp_clawServo
                clawServo.setPosition(unclamp_clawServo);
            }
            if(gamepad2.a) {
                //step_ferrisServo = 0.005
                ferrisServo.setPosition(ferrisServo.getPosition() + step_ferrisServo);
                //clawServo.setPosition(clawServo.getPosition() + 0.005);
            }
            //TODO this also opens the clawServo. odd
            if(gamepad2.b) {
                ferrisServo.setPosition(ferrisServo.getPosition() - step_ferrisServo);
                //clawServo.setPosition(clawServo.getPosition() - 0.005);
            }


            //TODO finish vertical slide ctrl
            if(mtrVertical.getCurrentPosition() < -vertSlideBottomBias) {
                //TODO enumerate joysticks
                powMtrVertical = (g2Joy[3] > 0 ? g2Joy[3] : 0) * vertMod;
            }
            //TODO consider getting rid of upper
            else if (mtrVertical.getCurrentPosition() > (vertSlideMax + vertSlideTopBias)) {
                powMtrVertical = (g2Joy[3] < 0 ? g2Joy[3] : 0) * vertMod;
            }
            else {
                powMtrVertical = g2Joy[3] * vertMod;
            }
            //setting powers of this guy in @see setPowers()



            //Reset IMU, takes 1s real time
            //TODO remind brandon that this is changed to dpad left
            if(gamepad1.dpad_up) {
                //TODO decide if we want to make the imu rotate back to 0 at the end of autons
                imuInit();
            }

            // foundation hook control
            if (gamepad1.dpad_up) {
                foundationServo.setPosition(
                        //foundationServo.getPosition() + 0.01
                        //up_foundation = 0.65
                        up_foundationServo
                );
            } else if(gamepad1.dpad_down) {
                foundationServo.setPosition(
                        //foundationServo.getPosition() - 0.01
                        //down_foundationServo = 0
                        down_foundationServo
                );
            }

            //TODO implement last-state control of intake
            // intake control - right bumper IN, left bumper OUT
            if (release && gamepad2.right_bumper) {
                if (intake == 0) {
                    leftRoller.setPower(intakeSpeed);
                    rightRoller.setPower(intakeSpeed);
                    intake = 1;
                } else {
                    leftRoller.setPower(0);
                    rightRoller.setPower(0);
                    intake = 0;
                }
                release = false;
                sleep(50);
            }

            if (release && gamepad2.left_bumper) {
                if (intake == 0) {
                    leftRoller.setPower(-intakeSpeed);
                    rightRoller.setPower(-intakeSpeed);
                    intake = -1;
                } else {
                    leftRoller.setPower(0);
                    rightRoller.setPower(0);
                    intake = 0;
                }
                release = false;
                sleep(50);
            }



            if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
                release = true;
            }



            //TODO finish telemetry outputs
            if(gamepad1.y)
                driveMode = DriveMode.CARTESIAN;
            else if(gamepad1.a)
                driveMode = DriveMode.FIELD;
            if(gamepad1.b)
                telemetryOutputType = TelemetryOutputType.DEBUG;
            else if (gamepad1.x)
                telemetryOutputType = TelemetryOutputType.SHORT;


            //applying the modifiers early will allow for mecanum ctrl calculations to not need to worry about scaling. still an ad hoc way to do it
            //TODO do proper scaling in the mecanum drive
            //applyModifiers();

            //joystick pows & set DriveMode -> motor pows
            driverControl();

            //send out pows to Drivetrain
            setPowers();

            //takes TelemetryOutputType and changes final telemetry accordingly
            telemetryStack();
        }
    }

    private void speedSwitch() {
        if(gamepad1.right_trigger>TRIGGER_DEADZONE){
            runFast = true;
            runSlow = false;
        }
        else if(gamepad1.left_trigger>TRIGGER_DEADZONE){
            runFast = false;
            runSlow = true;
        }
        if(runFast){
            modifier = speedSwitch[1];
        }
        if(runSlow){
            modifier = speedSwitch[0];
        }
    }

    public void initMotors() {
        leftRoller = hardwareMap.get(DcMotorEx.class, "leftRoller");
        rightRoller = hardwareMap.get(DcMotorEx.class, "rightRoller");

        leftRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRoller.setDirection(DcMotorSimple.Direction.REVERSE);

        ///////////////////////////////////////////////////////////////////////////////////////////

        mtrFL = hardwareMap.get(DcMotor.class, "fl");
        mtrFR = hardwareMap.get(DcMotor.class, "fr");
        mtrBL = hardwareMap.get(DcMotor.class, "bl");
        mtrBR = hardwareMap.get(DcMotor.class, "br");
        mtrVertical = hardwareMap.get(DcMotorEx.class, "vertical");

        mtrFL.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrFR.setDirection(DcMotorSimple.Direction.FORWARD);
        //backleft and backright are switched
        mtrBL.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrBR.setDirection(DcMotorSimple.Direction.FORWARD);
        //flipped from testing teleop
        mtrVertical.setDirection(DcMotorSimple.Direction.FORWARD);

        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mtrFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // encoder is reset to 0 at whatever starting position it is in
        mtrVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 0 br
        // 1 bl
        // 2 fr
        // 3 fl
    }

    private void initMotors2() {
        //d.init(hardwareMap);



        //telemetry.addData("Stat", "Finished drivetrain init!");



        leftRoller = hardwareMap.get(DcMotorEx.class, "leftRoller");
        rightRoller = hardwareMap.get(DcMotorEx.class, "rightRoller");
        mtrVertical = hardwareMap.get(DcMotorEx.class, "vertical");

        leftRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrVertical.setDirection(DcMotorSimple.Direction.REVERSE);

        //TODO what is default mode and zero pow behavior?
        mtrVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private void joystickArraysRefresh() {
        //TODO implement arraylists so that we can do the clean way : )
//        //TODO look at Gamepad.cleanMotionValues for scaling joystick vals after deadzone & clipping
//        Gamepad[] gamepads = {gamepad1, gamepad2};
//        double[][] joysticks = {g1Joy, g2Joy};
//
//        for(int i = 0; i<gamepads.length; i++) {
//            //TODO consider moving to byte[] (see Gamepad.toByteArray) for redundancy reduction
//            joysticks[i][0] = joystickAxisNormalization(gamepads[i].left_stick_x);
//            joysticks[i][1] = -joystickAxisNormalization(gamepads[i].left_stick_y);
//            joysticks[i][2] = joystickAxisNormalization(gamepads[i].right_stick_x);
//            joysticks[i][3] = -joystickAxisNormalization(gamepads[i].right_stick_y);
//        }

        g1Joy[0] = joystickAxisNormalization(gamepad1.left_stick_x);
        g1Joy[1] = -joystickAxisNormalization(gamepad1.left_stick_y);
        g1Joy[2] = joystickAxisNormalization(gamepad1.right_stick_x);
        g1Joy[3] = -joystickAxisNormalization(gamepad1.right_stick_y);

        g2Joy[0] = joystickAxisNormalization(gamepad2.left_stick_x);
        g2Joy[1] = -joystickAxisNormalization(gamepad2.left_stick_y);
        g2Joy[2] = joystickAxisNormalization(gamepad2.right_stick_x);
        g2Joy[3] = -joystickAxisNormalization(gamepad2.right_stick_y);

    }

    //TODO lambda functions/functional paradigms through kotlin to clean this up
    private double joystickAxisNormalization(double axisValue) {
        return ((Math.abs(axisValue) > joystickDeadzone ? axisValue : 0) * modifier);
    }

    @Deprecated
    private void applyModifiers() {
//        for(int i = 0; i<dtPowers.length; i++)
//            dtPowers[i]*=modifier;
        //need to make a refresh for dtPowers = {powFL...}
        powFL*=modifier;
        powFR*=modifier;
        powBR*=modifier;
        powBL*=modifier;
    }

    private void setPowers(){
        //d.setPowers(powFL, powFR, powBR, powBL);

        mtrFL.setPower(powFL);
        mtrFR.setPower(powFR);
        mtrBL.setPower(powBL);
        mtrBR.setPower(powBR);
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
        rotationServo.setPosition(init_rotationServo+rotationServoBias);
        ferrisServo.setPosition(init_ferrisServo);
        clawServo.setPosition(init_clawServo);
        //0.8 default
        foundationServo.setPosition(init_foundationServo);

        rotationServo.setDirection(Servo.Direction.FORWARD);
        ferrisServo.setDirection(Servo.Direction.FORWARD);
        foundationServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);
    }

    public void driverControl() {
        if(driveMode == DriveMode.FIELD) {
            double heading = getHeading();
            telemetry.addData("Heading", heading);

            heading = Math.toRadians(heading);

            if(heading > 0) {
                //ccw
                fwd = g1Joy[1] * Math.cos(Math.abs(heading)) - g1Joy[0] * Math.sin(Math.abs(heading));
                strafe = g1Joy[1] * Math.sin(Math.abs(heading)) + g1Joy[0] * Math.cos(Math.abs(heading));
            }
            else {
                //cw
                fwd = g1Joy[1] * Math.cos(Math.abs(heading)) + g1Joy[0] * Math.sin(Math.abs(heading));
                strafe = -g1Joy[1] * Math.sin(Math.abs(heading)) + g1Joy[0] * Math.cos(Math.abs(heading));
            }

            rotate = g1Joy[2];

            powFL = fwd + rotate + strafe;
            powFR = fwd - rotate - strafe;
            powBR = fwd - rotate + strafe;
            powBL = fwd + rotate - strafe;
        }
        else {
            // if the drive mode is Cartesian, we run the standard mecanum drive drive system
            powFL = g1Joy[1] + g1Joy[2] + g1Joy[0];
            powFR = g1Joy[1] - g1Joy[2] - g1Joy[0];
            powBR = g1Joy[1] - g1Joy[2] + g1Joy[0];
            powBL = g1Joy[1] + g1Joy[2] - g1Joy[0];
        }
    }

    public double getHeading() {
        refreshAngle();
        return heading;
    }

    public void refreshAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

        heading = AngleUnit.DEGREES.normalize(heading);

        //heading = Math.floor(heading);
        //heading = Range.clip(heading, -180.0, 180.0);
    }

    private void telemetryStack() {
        if(telemetryOutputType == TelemetryOutputType.SHORT) {

        }
        else if (telemetryOutputType == TelemetryOutputType.DEBUG) {
            //telemetry
            telemetry.addData("","");

        }

    }
    



}
