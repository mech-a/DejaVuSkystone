package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/***
 * 
 */


public class MecanumTeleOp extends LinearOpMode {

    DcMotor mtrHorizontal, mtrVertical;
    DcMotor mtrFL,mtrFR,mtrBL,mtrBR;
    CRServo sFrontRoller, sMiddleRoller;
    Servo sFrontIntake;

    double[] g1 = new double[4];

    double powFL, powFR, powBL, powBR;

    boolean frontRollerDirection = false,
            middleRollerDirection = false;


    @Override
    public void runOpMode() {
        initMotors();
        initServos();
        telemetry.addData("Stat", "Init!");
        telemetry.update();

        waitForStart();

        telemetry.addData("Controls", "x");
        
        if(gamepad1.x) {
            frontRollerDirection = !frontRollerDirection;
            sleep(25);
        }
        
        if(frontRollerDirection = false) {
            sFrontRoller.setPower(1);
        }
        
        else {
            sFrontRoller.setPower(-1);
        }

        if(gamepad1.y) {
            middleRollerDirection = !middleRollerDirection;
            sleep(25);
        }

        if(middleRollerDirection = false) {
            sMiddleRoller.setPower(1);
        }

        else {
            sMiddleRoller.setPower(-1);
        }
        
        if(gamepad1.a) {
            sFrontIntake.setPosition(sFrontIntake.getPosition()+0.01);
        }
        else if(gamepad1.b) {
            sFrontIntake.setPosition(sFrontIntake.getPosition()-0.01);
        }

        g1[0] = gamepad1.left_stick_x;
        g1[1] = -gamepad1.left_stick_y;
        g1[2] = gamepad1.right_stick_x;
        g1[3] = -gamepad1.right_stick_y;

        for(int i = 0; i <4; i++) {
            g1[i] /= 2;
        }

        powFL = g1[1] + g1[2] + g1[0];
        powFR = g1[1] - g1[2] - g1[0];
        powBL = g1[1] + g1[2] - g1[0];
        powBR = g1[1] - g1[2] + g1[0];

        telemetry.addData("Mtr powers", " " + powFL + powFR + powBL + powBR + " ");
        telemetry.addData("Front Roller Forward", frontRollerDirection);
        telemetry.addData("Middle Roller Forward", middleRollerDirection);
        telemetry.update();
        sleep(50);
    }

    public void initServos() {
        sFrontIntake = hardwareMap.get(Servo.class, "frontIntake");
        sFrontRoller = hardwareMap.get(CRServo.class, "frontRoller");
        sMiddleRoller = hardwareMap.get(CRServo.class, "middleRoller");

        sFrontIntake.setDirection(Servo.Direction.FORWARD);
        sFrontRoller.setDirection(CRServo.Direction.FORWARD);
        sMiddleRoller.setDirection(CRServo.Direction.FORWARD);

        sFrontIntake.setPosition(0);
    }
    
    public void initMotors() {
        mtrHorizontal = hardwareMap.get(DcMotor.class, "horizontal");
        mtrVertical = hardwareMap.get(DcMotor.class, "vertical");
        mtrFL = hardwareMap.get(DcMotor.class, "fl");
        mtrFR = hardwareMap.get(DcMotor.class, "fr");
        mtrBL = hardwareMap.get(DcMotor.class, "bl");
        mtrBR = hardwareMap.get(DcMotor.class, "br");

        mtrHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrFL.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrFR.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrBL.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrBR.setDirection(DcMotorSimple.Direction.REVERSE);
        
        mtrHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mtrHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
