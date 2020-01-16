package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StoneScorer implements Subassembly {
    DcMotorEx mtrVertical, leftRoller, rightRoller;
    Servo rotationServo, ferrisServo, clawServo, foundationServoL, foundationServoR;

    LinearOpMode caller;
    Telemetry telemetry;

    int intake = 0;
    boolean release = true;
    int extake_position = 0; // -1 for in and 1 for out

    ElapsedTime clawTimer = new ElapsedTime();

    @Override
    public void init() {
        mtrVertical = caller.hardwareMap.get(DcMotorEx.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[0]);
        leftRoller = caller.hardwareMap.get(DcMotorEx.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[1]);
        rightRoller = caller.hardwareMap.get(DcMotorEx.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[2]);

        rotationServo = caller.hardwareMap.get(Servo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[0]);
        ferrisServo = caller.hardwareMap.get(Servo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[1]);
        clawServo = caller.hardwareMap.get(Servo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[2]);
        foundationServoL = caller.hardwareMap.get(Servo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[3]);
        foundationServoR = caller.hardwareMap.get(Servo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[4]);

        // MOTORS SETUP ///////////////////////////////////////////////////////////////////////////

        leftRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRoller.setDirection(DcMotorSimple.Direction.REVERSE);

        mtrVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // encoder is reset to 0 at whatever starting position it is in
        mtrVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // SERVOS SETUP ///////////////////////////////////////////////////////////////////////////

        // initialization points for servos
        rotationServo.setPosition(0.66);
        ferrisServo.setPosition(0.32);
        clawServo.setPosition(1);
        foundationServoL.setPosition(0);
        foundationServoR.setPosition(0);

        rotationServo.setDirection(Servo.Direction.FORWARD);
        ferrisServo.setDirection(Servo.Direction.FORWARD);
        foundationServoR.setDirection(Servo.Direction.FORWARD);
        foundationServoL.setDirection(Servo.Direction.REVERSE);
        clawServo.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void status() {

    }

    public StoneScorer(LinearOpMode caller) {
        this.caller = caller;
        telemetry = caller.telemetry;
    }

    // set intake motors to intakePower
    public void intake(int intakePower) {
        leftRoller.setPower(intakePower);
        rightRoller.setPower(intakePower);
    }

    // extends extake
    public void extakeOut() {
        leftRoller.setPower(0);
        rightRoller.setPower(0);
        intake = 0;

        clawTimer.reset();
        extake_position = 1;
        clawServo.setPosition(0.62);

        if(extake_position == 1 && clawTimer.milliseconds() > 300) {
            ferrisServo.setPosition(0.86);      //ferris servo has limits 0.577 and 0.0522
            rotationServo.setPosition(0.00);  //rotation servo has limits 0.03 and 0.54
            extake_position = 0;
        }
    }

    // pull extake in
    public void extakeIn() {
        clawTimer.reset();
        extake_position = -1;
        clawServo.setPosition(1);
        ferrisServo.setPosition(0.6989);
        rotationServo.setPosition(0.66);

        if(extake_position == -1 && clawTimer.milliseconds() > 300) {
            ferrisServo.setPosition(0.32);
            extake_position = 0;
        }
    }

    // raise or lower vertical slide
    public void mtrVertical(int distance) {
        mtrVertical.setTargetPosition(distance);
        mtrVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (!caller.isStopRequested()) {
            mtrVertical.setPower(1);
        }

        if (!caller.isStopRequested()) {
            mtrVertical.setPower(0);
            mtrVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}