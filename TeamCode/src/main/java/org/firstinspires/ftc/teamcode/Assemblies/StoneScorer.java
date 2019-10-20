package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StoneScorer implements Subassembly {
    DcMotor mtrH, mtrV;
    CRServo frontRoller, roll1, roll2, roll3;
    Servo scooper, flipper, grabber;
    LinearOpMode caller;
    Telemetry telemetry;
    int speedH;

    @Override
    public void init() {
        mtrH = caller.hardwareMap.get(DcMotor.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[0]);
        mtrV = caller.hardwareMap.get(DcMotor.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[1]);

        frontRoller = caller.hardwareMap.get(CRServo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[0]);
        roll1 = caller.hardwareMap.get(CRServo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[1]);
        roll2 = caller.hardwareMap.get(CRServo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[2]);
        roll3 = caller.hardwareMap.get(CRServo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[3]);
        scooper = caller.hardwareMap.get(Servo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[4]);
        flipper = caller.hardwareMap.get(Servo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[5]);
        grabber = caller.hardwareMap.get(Servo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[6]);

    }

    @Override
    public void status() {

    }

    public void intake() {
        extendH(1);
        scoop(1);
        roll2(1);
        extendH(-1);
    }

    public void extake() {
        extendH(1);
        scoop(-1);
        roll2(-1);
        extendH(-1);
        scoop(1);
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

    public void scoop(int position) {
        scooper.setDirection(Servo.Direction.FORWARD);
        scooper.setPosition(position);
    }

    public void roll2(int direction) {
        frontRoller.setDirection(CRServo.Direction.FORWARD);
        roll1.setDirection(CRServo.Direction.FORWARD);

        frontRoller.setPower(direction);
        roll1.setPower(direction);
    }

    public void roll4(int dir) {
        frontRoller.setDirection(CRServo.Direction.FORWARD);
        roll1.setDirection(CRServo.Direction.FORWARD);
        roll2.setDirection(CRServo.Direction.FORWARD);
        roll3.setDirection(CRServo.Direction.FORWARD);

        frontRoller.setPower(dir);
        roll1.setPower(dir);
        roll2.setPower(dir);
        roll3.setPower(dir);
    }
}
