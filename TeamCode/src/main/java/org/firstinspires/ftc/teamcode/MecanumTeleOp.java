package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/***
 * 
 */

@TeleOp(name="Mecanum Non-dependency", group="Functionality")
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

        while (opModeIsActive()) {


            telemetry.addData("Controls", "x");

//            if (gamepad1.x) {
//                frontRollerDirection = !frontRollerDirection;
//                telemetry.addData("Loop", "in g1x");
//                sleep(200);
//            }
//
//            if (frontRollerDirection = false) {
//                sFrontRoller.setPower(1);
//            } else if (frontRollerDirection = true) {
//                sFrontRoller.setPower(-1);
//            }

//            if (gamepad1.y) {
//                middleRollerDirection = !middleRollerDirection;
//                telemetry.addData("Loop", "in g1y");
//                sleep(200);
//            }
//
//            if (middleRollerDirection = false) {
//                sMiddleRoller.setPower(1);
//            } else if(middleRollerDirection = true){
//                sMiddleRoller.setPower(-1);
//            }

            if (gamepad1.a) {
                sFrontIntake.setPosition(sFrontIntake.getPosition() + 0.01);
            } else if (gamepad1.b) {
                sFrontIntake.setPosition(sFrontIntake.getPosition() - 0.01);
            }
            else {


            }

            if (gamepad1.dpad_up) {
                mtrVertical.setPower(0.25);
            } else if (gamepad1.dpad_down) {
                mtrVertical.setPower(-0.25);
            } else {
                mtrVertical.setPower(0);
            }

            if (gamepad1.dpad_left) {
                mtrHorizontal.setPower(0.25);
            } else if (gamepad1.dpad_right) {
                mtrHorizontal.setPower(-0.25);
            }
            else {
                mtrHorizontal.setPower(0);
            }

            g1[0] = gamepad1.left_stick_x;
            g1[1] = -gamepad1.left_stick_y;
            g1[2] = gamepad1.right_stick_x;
            g1[3] = -gamepad1.right_stick_y;

            for (int i = 0; i < 4; i++) {
                g1[i] /= 2;
            }

            powFL = g1[1] + g1[2] + g1[0];
            powFR = g1[1] - g1[2] - g1[0];
            powBL = g1[1] + g1[2] - g1[0];
            powBR = g1[1] - g1[2] + g1[0];


            if(gamepad1.right_trigger > 0.2) {
                powFL = 1;
                powFR = 1;
                powBL = 1;
                powBR = 1;
            }

            mtrFL.setPower(powFL);
            mtrFR.setPower(powFR);
            mtrBL.setPower(powBL);
            mtrBR.setPower(powBR);

            //3 fl
            //2 fr
            //1 bl
            //0 br



            telemetry.addData("Mtr powers", " " + powFL + powFR + powBL + powBR +
                    mtrHorizontal.getPower() + mtrVertical.getPower() + " ");
            telemetry.addData("Front Roller Forward", sFrontIntake.getPosition());
            telemetry.addData("Front Roller Forward", frontRollerDirection);
            telemetry.addData("Middle Roller Forward", middleRollerDirection);
            telemetry.update();
            sleep(50);
        }
    }

    public void initServos() {
        sFrontIntake = hardwareMap.get(Servo.class, "frontIntake");
        sFrontRoller = hardwareMap.get(CRServo.class, "frontRoller");
        sMiddleRoller = hardwareMap.get(CRServo.class, "middleRoller");

        sFrontIntake.setDirection(Servo.Direction.FORWARD);
        sFrontRoller.setDirection(CRServo.Direction.FORWARD);
        sMiddleRoller.setDirection(CRServo.Direction.FORWARD);

        sFrontRoller.setPower(-1);
        sMiddleRoller.setPower(1);

        //sFrontIntake.setPosition(0);
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
            mtrFR.setDirection(DcMotorSimple.Direction.FORWARD);
            mtrBL.setDirection(DcMotorSimple.Direction.REVERSE);
            mtrBR.setDirection(DcMotorSimple.Direction.FORWARD);

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

            // 0 br
            // 1 bl
            // 2 fr
            // 3 fl
        }
    }
