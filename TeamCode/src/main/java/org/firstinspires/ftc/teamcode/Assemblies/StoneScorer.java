package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StoneScorer implements Subassembly {
    DcMotor mtrH, mtrV, linrA;
    CRServo frontRoller, roll1, roll2, roll3;
    Servo flipper, grabber;
    LinearOpMode caller;
    Telemetry telemetry;
    int speedH;
    public static final int distanceExtend = 4;
    public static final int rotateAmt = 3;

    @Override
    public void init() {
        mtrH = caller.hardwareMap.get(DcMotor.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[0]);
        mtrV = caller.hardwareMap.get(DcMotor.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[1]);
        linrA = caller.hardwareMap.get(DcMotor.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[2]);

        frontRoller = caller.hardwareMap.get(CRServo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[0]);
        roll1 = caller.hardwareMap.get(CRServo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[1]);
        roll2 = caller.hardwareMap.get(CRServo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[2]);
        roll3 = caller.hardwareMap.get(CRServo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[3]);
        flipper = caller.hardwareMap.get(Servo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[5]);
        grabber = caller.hardwareMap.get(Servo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[6]);

    }

    @Override
    public void status() {

    }

    public StoneScorer(LinearOpMode caller) {
        this.caller = caller;
        telemetry = caller.telemetry;
    }

    public void intake(int distExtendH, int dirRoll) {
        rotateH(rotateAmt);
        extendH(distExtendH);
        rotateH(-rotateAmt);
        roll2(dirRoll);
        extendH(-distExtendH);
    }

    public void extake(int distExtendH, int dirRoll) {
        rotateH(rotateAmt);
        extendH(distExtendH);
        // negative value entered here for dirRoll
        roll2(dirRoll);
        extendH(-distExtendH);
        rotateH(-rotateAmt);
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
        frontRoller.setDirection(CRServo.Direction.FORWARD);
        roll1.setDirection(CRServo.Direction.FORWARD);

        frontRoller.setPower(direction);
        roll1.setPower(direction);
    }

    public void roll4(int direction) {
        frontRoller.setDirection(CRServo.Direction.FORWARD);
        roll1.setDirection(CRServo.Direction.FORWARD);
        roll2.setDirection(CRServo.Direction.FORWARD);
        roll3.setDirection(CRServo.Direction.FORWARD);

        frontRoller.setPower(direction);
        roll1.setPower(direction);
        roll2.setPower(direction);
        roll3.setPower(direction);
    }

    public void rotateH(int distance) {
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

    public void hookFoundation(int hookDir) {
        extendH(distanceExtend);
        if(hookDir == 1) {
            rotateH(rotateAmt);
        } else {
            rotateH(-rotateAmt);
        }
        extendH(-distanceExtend);
    }
}
