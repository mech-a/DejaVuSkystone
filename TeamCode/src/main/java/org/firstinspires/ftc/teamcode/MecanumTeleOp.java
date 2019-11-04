package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;

/***
 *
 */

@TeleOp(name="Mecanum Non-dependency", group="Functionality")
public class MecanumTeleOp extends LinearOpMode {

    DcMotor mtrHorizontal, mtrVertical, mtrIntake, mtrArmLift;
    DcMotor mtrFL,mtrFR,mtrBL,mtrBR;
    CRServo sFrontRoller, sMiddleRoller;
    Servo servoHand, servoArm;
    //Servo sFrontIntake;  < this servo was replaced by the DCMotor mtrIntake

    double fwd, strafe, rotate;

    public enum DriveMode {
        FIELD, CARTESIAN
    }

    double[] speedSwitch = {0.05,0.375};
    boolean runFast = true, runSlow = false;
    double modifier = speedSwitch[1];
    static double DEADZONE = 0.15, TRIGGER_DEADZONE = 0.1;

    //Keep default as field or cartesian?
    DriveMode driveMode = DriveMode.FIELD;

    private BNO055IMU imu;
    private BNO055IMU.Parameters gyroParameters;
    private double heading;

    private Orientation angles;

    boolean gripper = false;
    double[] g1 = new double[4];

    double powFL, powFR, powBL, powBR;


    final double HORIZONTAL_MAX = 2600;
    final double HORIZONTAL_MIN = 0;

    //top is -2332
    // bottom is 800
    final double ARMLIFT_MIN = 0; //top
    final double ARMLIFT_MAX = 4000; //bottom

    final double VERTICAL_MIN = -2900;
    final double VERTICAL_MAX = 0;


    // arm 5773 horiontal
    // lift 953

    // lift max is -827

    boolean frontRollerDirection = false,
            middleRollerDirection = false;

    StoneScorer ss = new StoneScorer(this);

    @Override
    public void runOpMode() {
        initMotors();
        initServos();
        ss.init();
        imuInit();

        telemetry.addData("Stat", "Init!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // all the way out is 0.55
            // middle position is 0.23
            // all the one down is 0.17

            // set vertical lift to -360
            if (gamepad2.dpad_down) {
                servoArm.setPosition(0.20);
                ss.liftV(0);
            } else if (gamepad2.dpad_up) {
                ss.liftV(-700);
                servoArm.setPosition(0.23);
            } else if (gamepad2.dpad_right) {
                servoArm.setPosition(0.59);
            }

            if (gamepad2.x && (servoArm.getPosition() > 0)) {
                // when x is hit, then the gripper clamp moves in more (tigher grip)
                servoArm.setPosition(servoArm.getPosition() - 0.005);
            } else if (gamepad2.y && (servoArm.getPosition() < 1)) {
                // when y is hit, then the gripper clamp moves out more (looser grip)
                servoArm.setPosition(servoArm.getPosition() + 0.005);
            }


            if (gamepad2.right_trigger > 0.2) {
                servoHand.setPosition(0.20); //in
            } else if (gamepad2.left_trigger > 0.2) {
                servoHand.setPosition(0); //out
            }

            if(gamepad2.dpad_down) {
                //servoArm.setPosition(0); //move the arm all the way down
                servoArm.setPosition(servoArm.getPosition()-0.01);
                //TODO move the vertical slide all the way down
            } else if(gamepad2.dpad_up) {
                servoArm.setPosition(servoArm.getPosition()+0.01);
                //servoArm.setPosition(0.25); //move the servo so the top of gripper is parallel to ground
                //TODO move the vertical slide slightly up
            } else if(gamepad2.dpad_right) {
                servoHand.setPosition(servoHand.getPosition()+0.01);
                //servoArm.setPosition(0.5); //move the servo so the arm channel is parallel to the ground
            } else if(gamepad2.dpad_left) {
                servoHand.setPosition(servoHand.getPosition()- 0.01);
            }

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

            if(gamepad1.dpad_up) {
                //todo figure out reset angle
                imuInit();
            }

            g1[0] = gamepad1.left_stick_x;
            g1[1] = -gamepad1.left_stick_y;
            g1[2] = gamepad1.right_stick_x;
            g1[3] = -gamepad1.right_stick_y;

            for(int i = 0; i < g1.length; i++)
                g1[i] = (Math.abs(g1[i]) > DEADZONE ? g1[i] : 0) * modifier;

            if(gamepad1.b) {
                driveMode = DriveMode.CARTESIAN;
            }
            else if (gamepad1.a) {
                driveMode = DriveMode.FIELD;
            }


            driverControl();


//            for (int i = 0; i < 4; i++) {
//                g1[i] /= 2;
//            }

//            powFL = g1[1] + g1[2] + g1[0];
//            powFR = g1[1] - g1[2] - g1[0];
//            powBL = g1[1] + g1[2] - g1[0];
//            powBR = g1[1] - g1[2] + g1[0];
//
//            if(gamepad1.right_trigger > 0.2) {
//                powFL = 1;
//                powFR = 1;
//                powBL = 1;
//                powBR = 1;
//            }

            //TODO make speed switches

            mtrFL.setPower(powFL);
            mtrFR.setPower(powFR);
            mtrBL.setPower(powBL);
            mtrBR.setPower(powBR);

            if(gamepad2.left_stick_y > 0.1 && mtrArmLift.getCurrentPosition() < ARMLIFT_MAX) {
                mtrArmLift.setPower(gamepad2.left_stick_y / 2);
            } else if (gamepad2.left_stick_y < -0.1 && mtrArmLift.getCurrentPosition() > ARMLIFT_MIN) {
                mtrArmLift.setPower(gamepad2.left_stick_y / 2);
            } else {
                mtrArmLift.setPower(0);
            }


//            if((gamepad2.left_stick_y > 0.1) || (gamepad2.left_stick_y < -0.1)) {
//                mtrArmLift.setPower(gamepad2.left_stick_y/2);
//                telemetry.addData("lin act", "1");
//                telemetry.update();
//            } else {
//                mtrArmLift.setPower(0);
//                telemetry.addData("lin act", "0");
//                telemetry.update();
//            }

            //////////////



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
            telemetry.addData("grippy man:", servoHand.getPosition());
            telemetry.addData("big arm dude: ", servoArm.getPosition());
            telemetry.update();
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

    private void imuInit() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyroParameters.loggingEnabled      = true;
        gyroParameters.loggingTag          = "IMU";

        //Default is 32
        //TODO check powerdrain
        gyroParameters.gyroBandwidth = BNO055IMU.GyroBandwidth.HZ523;

        imu.initialize(gyroParameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void refreshAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

        heading = AngleUnit.DEGREES.normalize(heading);


//        heading = Math.floor(heading);
//        heading = Range.clip(heading, -180.0, 180.0);
    }

    public double getHeading() {
        refreshAngle();
        return heading;
    }

    public void driverControl() {
        if(driveMode == DriveMode.FIELD) {
            double heading = getHeading();
            telemetry.addData("Heading", heading);

            heading = Math.toRadians(heading);


            if(heading > 0) {
                //ccw
                fwd = g1[1] * Math.cos(Math.abs(heading)) - g1[0] * Math.sin(Math.abs(heading));
                strafe = g1[1] * Math.sin(Math.abs(heading)) + g1[0] * Math.cos(Math.abs(heading));
            }
            else {
                //cw
                fwd = g1[1] * Math.cos(Math.abs(heading)) + g1[0] * Math.sin(Math.abs(heading));
                strafe = -g1[1] * Math.sin(Math.abs(heading)) + g1[0] * Math.cos(Math.abs(heading));
            }

            rotate = g1[2];

            powFL = fwd + rotate + strafe;
            powFR = fwd - rotate - strafe;
            powBL = fwd + rotate - strafe;
            powBR = fwd - rotate + strafe;

        }
        else {
            //If the drive mode is Cartesian, we run the standard mecanum drive drive system

            powFL = g1[1] + g1[2] + g1[0];
            powFR = g1[1] - g1[2] - g1[0];
            powBL = g1[1] + g1[2] - g1[0];
            powBR = g1[1] - g1[2] + g1[0];

        }

    }

    public void initServos() {

        //sFrontIntake = hardwareMap.get(Servo.class, "frontIntake");
        sFrontRoller = hardwareMap.get(CRServo.class, "frontRoller");
        sMiddleRoller = hardwareMap.get(CRServo.class, "middleRoller");
        servoArm = hardwareMap.get(Servo.class, "arm_servo");
        servoHand = hardwareMap.get(Servo.class, "gripper_servo");
        // initialize servo hand position
        servoHand.setPosition(0);
        // initialize arm servo position
        servoArm.setPosition(0.20);


        //sFrontIntake.setDirection(Servo.Direction.FORWARD);
        sFrontRoller.setDirection(CRServo.Direction.REVERSE);
        sMiddleRoller.setDirection(CRServo.Direction.FORWARD);

        //sFrontIntake.setPosition(0);
    }

    public void initMotors() {
        mtrHorizontal = hardwareMap.get(DcMotor.class, "horizontal");
        mtrVertical = hardwareMap.get(DcMotor.class, "vertical");
        mtrArmLift = hardwareMap.get(DcMotor.class, "linearActuator");
        mtrIntake = hardwareMap.get(DcMotor.class, "intake");

        mtrHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrArmLift.setDirection(DcMotorSimple.Direction.FORWARD);
        mtrIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        mtrFL = hardwareMap.get(DcMotor.class, "fl");
        mtrFR = hardwareMap.get(DcMotor.class, "fr");
        mtrBL = hardwareMap.get(DcMotor.class, "bl");
        mtrBR = hardwareMap.get(DcMotor.class, "br");

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

        mtrHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
