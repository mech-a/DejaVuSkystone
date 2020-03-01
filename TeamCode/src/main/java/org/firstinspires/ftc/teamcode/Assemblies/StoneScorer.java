package org.firstinspires.ftc.teamcode.Assemblies;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.Assemblies.Constants.unclampClawServo;
import static org.firstinspires.ftc.teamcode.Assemblies.Constants.intakeFerrisServo;
import static org.firstinspires.ftc.teamcode.Assemblies.Constants.frontRotationServo;
import static org.firstinspires.ftc.teamcode.Assemblies.Constants.clampClawServoPosition;
import static org.firstinspires.ftc.teamcode.Assemblies.Constants.placeFerrisServoPosition;
import static org.firstinspires.ftc.teamcode.Assemblies.Constants.backRotationServoPosition;
import static org.firstinspires.ftc.teamcode.Assemblies.Constants.initClawServo;
import static org.firstinspires.ftc.teamcode.Assemblies.Constants.initRotationServo;
import static org.firstinspires.ftc.teamcode.Assemblies.Constants.initFerrisServo;
import static org.firstinspires.ftc.teamcode.Assemblies.Constants.leftFoundationDown;
import static org.firstinspires.ftc.teamcode.Assemblies.Constants.leftFoundationUp;
import static org.firstinspires.ftc.teamcode.Assemblies.Constants.mtrVerticalStop1;
import static org.firstinspires.ftc.teamcode.Assemblies.Constants.mtrVerticalStop2;
import static org.firstinspires.ftc.teamcode.Assemblies.Constants.rightFoundationDown;
import static org.firstinspires.ftc.teamcode.Assemblies.Constants.rightFoundationUp;

public class StoneScorer //implements Subassembly
{

    private static final int NUM_DCMOTORS = 3;
    private static final int NUM_SERVOS = 5;

    ArrayList<DcMotor> dcMotors;
    ArrayList<Servo> servos;

    LinearOpMode caller;
    HardwareMap hwMap;
    Telemetry telemetry;

    private boolean isInitialized = false;

    public enum DCMotorOrientation {
        // 0, 1, 2, 3
        mtrVertical(0),leftRoller(1),rightRoller(2);

        private int order;

        DCMotorOrientation(int anOrder) {
            order = anOrder;
        }

        int getOrder() {
            return order;
        }
    }

    public enum ServoMotorOrientation {
        // 0, 1, 2, 3
        rotationServo(0),ferrisServo(1),clawServo(2), leftFoundationServo(3), rightFoundationServo(4);

        private int order;

        ServoMotorOrientation(int anOrder) {
            order = anOrder;
        }

        int getOrder() {
            return order;
        }
    }

    boolean release = true;

    int intake = 0;
    int extake_position = 0; // -1 for in and 1 for out

    ElapsedTime clawTimer = new ElapsedTime();

    @Deprecated
    public void init() {

    }


    public void init(HardwareMap aHwMap) {
        hwMap = aHwMap;
        if(hwMap == null)
            telemetry.addData("Stat", "Null HWMAP");
        telemetry.update();

        //FOR DC MOTORS
        dcMotors = new ArrayList<>();
        for(int i = 0; i<NUM_DCMOTORS; i++) {
            DcMotor temp = null;

            try {
                temp = hwMap.get(DcMotor.class, ConfigurationData.BLOCK_MANIPULATOR_MOTOR_NAMES[i]);
            } catch (NullPointerException e) {
                Log.d("NPE", "DRIVETRAIN MOTOR ACCESS FAILED");
            }
            //temp = hwMap.get(DcMotor.class, ConfigurationData.DRIVETRAIN_MOTOR_NAMES[i]);

            if (i == 0) {
                temp.setDirection(DcMotor.Direction.REVERSE);
                temp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                temp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                temp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (i == 1) {
                temp.setDirection(DcMotor.Direction.FORWARD);
            }
            else {
                temp.setDirection(DcMotor.Direction.REVERSE);
            }

            dcMotors.add(temp);
        }

        //FOR SERVOS
        servos = new ArrayList<>();
        for(int i = 0; i<NUM_SERVOS; i++) {
            Servo temp = null;

            try {
                temp = hwMap.get(Servo.class, ConfigurationData.BLOCK_MANIPULATOR_SERVO_NAMES[i]);
            } catch (NullPointerException e) {
                Log.d("NPE", "DRIVETRAIN MOTOR ACCESS FAILED");
            }

            temp.setDirection(Servo.Direction.FORWARD);

            if (i == 0)
                temp.setPosition(initRotationServo);
            else if (i == 1)
                temp.setPosition(initFerrisServo);
            else if (i == 2)
                temp.setPosition(initClawServo);
            else if(i == 3)
                temp.setPosition(leftFoundationUp);
            else
                temp.setPosition(rightFoundationUp);

            servos.add(temp);
        }
        isInitialized = true;
    }

    public void status() {

    }

    //TODO switch out functions in roadrunner based autons w this
    public void setBlock(double d, double d2) {

    }


    public StoneScorer(LinearOpMode caller) {
        this.caller = caller;
        hwMap = caller.hardwareMap;
        telemetry = caller.telemetry;
    }

    // set intake motors to intakePower
    public void intake(double intakePower) {
        dcMotors.get(1).setPower(intakePower);
        dcMotors.get(2).setPower(intakePower);
    }

    // extends extake
    public void extakeOut() {
        dcMotors.get(1).setPower(0);
        dcMotors.get(2).setPower(0);
        intake = 0;

        clawTimer.reset();
        extake_position = 1;
        servos.get(2).setPosition(clampClawServoPosition);
        try {
            sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if(extake_position == 1 //&& clawTimer.milliseconds() > 300
        ) {
            servos.get(1).setPosition(placeFerrisServoPosition);      //ferris servo has limits 0.577 and 0.0522
            servos.get(0).setPosition(backRotationServoPosition);  //rotation servo has limits 0.03 and 0.54
            extake_position = 0;
        }
    }

    public void extakeOutPartial() {
        dcMotors.get(1).setPower(0);
        dcMotors.get(2).setPower(0);
        intake = 0;

        clawTimer.reset();
        extake_position = 1;
        servos.get(2).setPosition(clampClawServoPosition);
        try {
            sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if(extake_position == 1 //&& clawTimer.milliseconds() > 300
        ) {
            servos.get(1).setPosition(.9);   //.85   //ferris servo has limits 0.577 and 0.0522
            servos.get(0).setPosition(.3);  //28.8/270 +.02 //rotation servo has limits 0.03 and 0.54
            extake_position = 0;
        }
    }

    public void dropStone() {
        servos.get(2).setPosition(unclampClawServo);
    }

    public void clampStone() { servos.get(2).setPosition(clampClawServoPosition);}

    // pull extake in
    public void extakeIn() {
        clawTimer.reset();
        extake_position = -1;
        servos.get(2).setPosition(unclampClawServo);
        servos.get(1).setPosition(intakeFerrisServo);
        servos.get(0).setPosition(frontRotationServo);

        try {
            sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if(extake_position == -1 //&& clawTimer.milliseconds() > 300
        ) {
            servos.get(1).setPosition(intakeFerrisServo);
            extake_position = 0;
        }
    }

    // lower servo to hook foundation
    // TODO: FIND AND PUT IN CORRECT VALUES
    public void hookFoundation() {
        servos.get(3).setPosition(leftFoundationDown);
        servos.get(4).setPosition(rightFoundationDown);
    }

    // raise servo to unhook foundation
    public void unhookFoundation() {
        servos.get(3).setPosition(leftFoundationUp);
        servos.get(4).setPosition(rightFoundationUp);
    }

    // raise or lower vertical slide
    public void mtrVertical(int distance) {
        dcMotors.get(0).setTargetPosition(distance);
        dcMotors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (!caller.isStopRequested()) {
            dcMotors.get(0).setPower(mtrVerticalStop1);
        }

        if (!caller.isStopRequested()) {
            dcMotors.get(0).setPower(mtrVerticalStop2);
            dcMotors.get(0).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}