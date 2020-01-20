package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;

public class StoneScorer implements Subassembly {
    DcMotor mtrH, mtrV, linrA, frontRoller;
    CRServo roll1, roll2, roll3;
    Servo flipper, grabber;
    LinearOpMode caller;
    Telemetry telemetry;
    public static double speedH = 0.5;
    public static final int distanceExtend = 2400; // was 1600

    @Override
    public void init() {
        mtrH = caller.hardwareMap.get(DcMotor.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[0]);
        mtrV = caller.hardwareMap.get(DcMotor.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[1]);
        linrA = caller.hardwareMap.get(DcMotor.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[2]);
        frontRoller = caller.hardwareMap.get(DcMotor.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[3]);

        roll1 = caller.hardwareMap.get(CRServo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[0]);
        roll2 = caller.hardwareMap.get(CRServo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[1]);
        //flipper = caller.hardwareMap.get(Servo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[2]);
        //grabber = caller.hardwareMap.get(Servo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[3]);

        mtrH.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrV.setDirection(DcMotorSimple.Direction.REVERSE);
        linrA.setDirection(DcMotorSimple.Direction.FORWARD );
        frontRoller.setDirection(DcMotorSimple.Direction.REVERSE);

        mtrH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrV.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linrA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRoller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mtrH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrV.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linrA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flipper = caller.hardwareMap.get(Servo.class, "arm_servo");
        grabber = caller.hardwareMap.get(Servo.class, "gripper_servo");

        flipper.setPosition(0.21);
    }

    @Override
    public void status() {

    }

    public StoneScorer(LinearOpMode caller) {
        this.caller = caller;
        telemetry = caller.telemetry;
    }

<<<<<<< Updated upstream
    // extend the horizontal to parameter value, lower the horizontal slide to parameter value
    public void setBlock(int extendHVal, int dropVal) {
        extendH(extendHVal);
        liftH(dropVal);
=======
    // set intake motors to intakePower
    public void intake(double intakePower) {
        leftRoller.setPower(intakePower);
        rightRoller.setPower(intakePower);
>>>>>>> Stashed changes
    }

    // start rolling the first two intake wheels, retract the horizontal,
    public void intake(int dirRoll, int retractHVal) {
        roll2(dirRoll);
        extendH(retractHVal / 2);
        liftH(-300);
        extendH(retractHVal / 4);
        liftH(-200);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void extake(int extendHVal, int liftVal, int dirRoll, int retractHVal) {
        extendH(extendHVal);
        liftH(liftVal);

        // negative value entered here for dirRoll
        roll2(dirRoll);
        try {
            sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
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
            telemetry.update();
        }

        if (!caller.isStopRequested()) {
            mtrH.setPower(0);
            mtrH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void roll2(int direction) {
        frontRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        roll1.setDirection(CRServo.Direction.FORWARD);

        if (direction == 1) {
            frontRoller.setPower(direction * 0.50);
            roll1.setPower(direction);
        } else if (direction == -1) {
            frontRoller.setPower(direction);
            roll1.setDirection(CRServo.Direction.REVERSE);
            roll1.setPower(1);
        } else if (direction == 0) {
            frontRoller.setPower(0);
            roll1.setPower(0);
        }


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

    // resets linear actuator motor position to 0, runs to position
    public void liftH(int position) {
        linrA.setTargetPosition(position);
        linrA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linrA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (!caller.isStopRequested()) {
            linrA.setPower(speedH);
        }

        while (!caller.isStopRequested() && (linrA.isBusy())) {
            //TODO change telemetry name to enum
            telemetry.addData("mtrHorizontal", "%7d : %7d",
                    linrA.getCurrentPosition(), position);
        }

        if (!caller.isStopRequested()) {
            linrA.setPower(0);
            linrA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void liftV(int distance) {
        mtrV.setTargetPosition(distance);
        mtrV.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrV.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (!caller.isStopRequested()) {
            mtrV.setPower(speedH);
        }

        while (!caller.isStopRequested() && (linrA.isBusy())) {
            //TODO change telemetry name to enum
            telemetry.addData("mtrHorizontal", "%7d : %7d",
                    mtrV.getCurrentPosition(), distance);
        }

        if (!caller.isStopRequested()) {
            mtrV.setPower(0);
            mtrV.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void hookFoundation(int hookDir, int liftVal) {
        // 1 if it is hooking on
        // anything but 1 if it is releasing the foundation
        if(hookDir == 1) {
            // extend the horizontal slide to 2400
            extendH(distanceExtend);
            liftH(liftVal);
        } else {
            liftH(-liftVal);
            // bring in the horizontal slide
            extendH(-distanceExtend + 600);
        }

    }
}
