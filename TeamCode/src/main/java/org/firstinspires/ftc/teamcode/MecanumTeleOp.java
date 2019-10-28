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

    DcMotor mtrHorizontal, mtrVertical, mtrIntake, mtrArmLift;
    DcMotor mtrFL,mtrFR,mtrBL,mtrBR;
    CRServo sFrontRoller, sMiddleRoller;
    //Servo sFrontIntake;  < this servo was replaced by the DCMotor mtrIntake

    double[] g1 = new double[4];

    double powFL, powFR, powBL, powBR;

    final double HORIZONTAL_MAX = 2300; //2407 for auton
    final double HORIZONTAL_MIN = 0;
    final double ARMLIFE_MAX = -3951; //-5302, -5491 for auon
    final double ARMLIFT_MIN = -7500;
    final double VERTICAL_MIN = -3050;
    final double VERTICAL_MAX = -150;

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

            if (gamepad1.x) {
                mtrIntake.setPower(1);
                sFrontRoller.setPower(-1);
                sMiddleRoller.setPower(1);
            } else if (gamepad1.y) {
                mtrIntake.setPower(0);
                sFrontRoller.setPower(0);
                sMiddleRoller.setPower(0);
            }
            else {


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

            //TODO make speed switches

            mtrFL.setPower(powFL);
            mtrFR.setPower(powFR);
            mtrBL.setPower(powBL);
            mtrBR.setPower(powBR);

            if((gamepad2.left_stick_y > 0.1 && mtrArmLift.getCurrentPosition() < ARMLIFE_MAX) ||
                    (gamepad2.left_stick_y < -0.1 && mtrArmLift.getCurrentPosition() > ARMLIFT_MIN)) {
                mtrArmLift.setPower(gamepad2.left_stick_y/2);
            } else {
                mtrArmLift.setPower(0);
            }

            if((gamepad2.left_stick_x > 0.1 && mtrHorizontal.getCurrentPosition() < HORIZONTAL_MAX) ||
                    (gamepad2.left_stick_x < -0.1 && mtrHorizontal.getCurrentPosition() > HORIZONTAL_MIN)) {
                mtrHorizontal.setPower(gamepad2.left_stick_x/2);
            } else {
                mtrHorizontal.setPower(0);
            }
            if((gamepad2.right_stick_y > 0.1 && mtrVertical.getCurrentPosition() < VERTICAL_MAX) ||
                    (gamepad2.right_stick_y < -0.1 && mtrVertical.getCurrentPosition() > VERTICAL_MIN)) {
                mtrVertical.setPower(gamepad2.right_stick_y/1.5);
            } else {
                mtrVertical.setPower(0);
            }


            //3 fl
            //2 fr
            //1 bl
            //0 br



            telemetry.addData("Mtr powers", " " + powFL + powFR + powBL + powBR +
                    mtrHorizontal.getPower() + mtrVertical.getPower() + " ");
            //telemetry.addData("Front Roller Forward", sFrontIntake.getPosition());
//            telemetry.addData("Front Roller Forward", frontRollerDirection);
//            telemetry.addData("Middle Roller Forward", middleRollerDirection);
            telemetry.addData("vertical lift", mtrVertical.getCurrentPosition());
            telemetry.addData("horizontal arm", mtrHorizontal.getCurrentPosition());
            telemetry.addData("arm lift", mtrArmLift.getCurrentPosition());
            telemetry.update();
            sleep(50);
        }
    }

    public void initServos() {
        //sFrontIntake = hardwareMap.get(Servo.class, "frontIntake");
        sFrontRoller = hardwareMap.get(CRServo.class, "frontRoller");
        sMiddleRoller = hardwareMap.get(CRServo.class, "middleRoller");

        //sFrontIntake.setDirection(Servo.Direction.FORWARD);
        sFrontRoller.setDirection(CRServo.Direction.REVERSE);
        sMiddleRoller.setDirection(CRServo.Direction.FORWARD);

//        sFrontRoller.setPower(-1);
//        sMiddleRoller.setPower(1);

        //sFrontIntake.setPosition(0);
    }
    
    public void initMotors() {
            mtrHorizontal = hardwareMap.get(DcMotor.class, "horizontal");
            mtrVertical = hardwareMap.get(DcMotor.class, "vertical");
            mtrArmLift = hardwareMap.get(DcMotor.class, "linearActuator");
            mtrIntake = hardwareMap.get(DcMotor.class, "intake");
            mtrFL = hardwareMap.get(DcMotor.class, "fl");
            mtrFR = hardwareMap.get(DcMotor.class, "fr");
            mtrBL = hardwareMap.get(DcMotor.class, "bl");
            mtrBR = hardwareMap.get(DcMotor.class, "br");

            mtrHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
            mtrVertical.setDirection(DcMotorSimple.Direction.REVERSE);
            mtrArmLift.setDirection(DcMotorSimple.Direction.FORWARD );
            mtrIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            mtrFL.setDirection(DcMotorSimple.Direction.REVERSE);
            mtrFR.setDirection(DcMotorSimple.Direction.FORWARD);
            mtrBL.setDirection(DcMotorSimple.Direction.REVERSE);
            mtrBR.setDirection(DcMotorSimple.Direction.FORWARD);

            mtrHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mtrVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mtrIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mtrArmLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            mtrHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtrVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtrArmLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtrIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtrFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtrFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtrBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtrBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // 0 br
            // 1 bl
            // 2 fr
            // 3 fl
        }
    }
