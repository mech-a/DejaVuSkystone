package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StoneScorer implements Subassembly {
    DcMotor mtrH, mtrV, linrA, frontRoller;
    CRServo roll1, roll2, roll3;
    Servo flipper, grabber;
    LinearOpMode caller;
    Telemetry telemetry;
    int speedH;
    public static final int distanceExtend = 4;

    @Override
    public void init() {
        mtrH = caller.hardwareMap.get(DcMotor.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[0]);
        mtrV = caller.hardwareMap.get(DcMotor.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[1]);
        linrA = caller.hardwareMap.get(DcMotor.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[2]);
        frontRoller = caller.hardwareMap.get(DcMotor.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[3]);

        roll1 = caller.hardwareMap.get(CRServo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[1]);
        roll2 = caller.hardwareMap.get(CRServo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[2]);
        roll3 = caller.hardwareMap.get(CRServo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[3]);
        flipper = caller.hardwareMap.get(Servo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[5]);
        grabber = caller.hardwareMap.get(Servo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[6]);

        mtrH.setDirection(DcMotorSimple.Direction.FORWARD);
        mtrV.setDirection(DcMotorSimple.Direction.FORWARD);
        linrA.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRoller.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void status() {

    }

    public StoneScorer(LinearOpMode caller) {
        this.caller = caller;
        telemetry = caller.telemetry;
    }

    // extend the horizontal to parameter value, lower the horizontal slide to parameter value
    public void setBlock(int extendHVal, int liftVal) {
        extendH(extendHVal);
        liftH(liftVal);
    }

    // start rolling the first two intake wheels, retract the horizontal,
    public void intake(int dirRoll, int retractHVal) {
        roll2(dirRoll);
        extendH(retractHVal);
    }

    public void extake(int liftVal, int extendHVal, int dirRoll, int retractHVal) {
        liftH(liftVal);
        extendH(extendHVal);
        // negative value entered here for dirRoll
        roll2(dirRoll);
        extendH(retractHVal);
    }

    // used for both extending and retracting the horizontal slides
    public void extendH(int distance) {
        mtrH.setTargetPosition(distance);
        mtrH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrH.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (!caller.isStopRequested()) {
            mtrH.setPower(speedH);
        }

        while (!caller.isStopRequested() && (mtrH.isBusy())) {
            //TODO change telemetry name to enum
            telemetry.addData("mtrHorizontal", "%7d : %7d",
                    mtrH.getCurrentPosition(), distance);
        }

        if (!caller.isStopRequested()) {
            mtrH.setPower(0);
            mtrH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void roll2(int direction) {
        frontRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        roll1.setDirection(CRServo.Direction.FORWARD);

        frontRoller.setPower(direction * 0.50);
        roll1.setPower(direction);
    }

    public void roll4(int direction) {
        frontRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        roll1.setDirection(CRServo.Direction.FORWARD);
        roll2.setDirection(CRServo.Direction.FORWARD);
        roll3.setDirection(CRServo.Direction.FORWARD);

        frontRoller.setPower(direction * 0.50);
        roll1.setPower(direction);
        roll2.setPower(direction);
        roll3.setPower(direction);
    }

    public void liftH(int distance) {
        linrA.setTargetPosition(distance);
        linrA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linrA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (!caller.isStopRequested()) {
            linrA.setPower(speedH);
        }

        while (!caller.isStopRequested() && (linrA.isBusy())) {
            //TODO change telemetry name to enum
            telemetry.addData("mtrHorizontal", "%7d : %7d",
                    linrA.getCurrentPosition(), distance);
        }

        if (!caller.isStopRequested()) {
            linrA.setPower(0);
            linrA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void hookFoundation(int hookDir, int liftVal) {
        extendH(distanceExtend);
        if(hookDir == 1) {
            liftH(liftVal);
        } else {
            liftH(-liftVal);
        }
        extendH(-distanceExtend);
    }
}
