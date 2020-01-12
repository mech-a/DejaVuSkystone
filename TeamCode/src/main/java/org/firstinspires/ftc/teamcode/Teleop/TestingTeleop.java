package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;

@TeleOp(name="TEST Intake/DT Teleop", group="Functionality")
public class TestingTeleop extends LinearOpMode {

    DcMotor mtrFL,mtrFR,mtrBL,mtrBR;
    Servo rotationServo, ferrisServo, foundationServoL, foundationServoR;
    Servo clawServo;
    DcMotorEx mtrVertical, leftRoller, rightRoller;

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

    double[] g1 = new double[4];

    double powFL, powFR, powBL, powBR;

    double intakeSpeed = 0.75;
    // top is -2700
    // bottom is starting, which is 0
    final double VERTICAL_MIN = -5000;
    final double VERTICAL_MAX = 5000;

    int intake = 0;

    StoneScorer ss = new StoneScorer(this);

    @Override
    public void runOpMode() {
        initMotors();
        initServos();
        imuInit();

        telemetry.addData("Stat", "Init!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            mtrVertical.setPower(gamepad1.right_stick_y / 2);

            if(gamepad2.x) {
                extakeSetup();
            }
            else if(gamepad2.y) {
                extakeReturn();
            }

            if(gamepad1.a) {
                rotationServo.setPosition(rotationServo.getPosition() + 0.005);
                //clawServo.setPosition(clawServo.getPosition() + 0.005);
            }

            if(gamepad1.b) {
                rotationServo.setPosition(rotationServo.getPosition() - 0.005);
                //clawServo.setPosition(clawServo.getPosition() - 0.005);
            }

            if(gamepad2.a) {
                ferrisServo.setPosition(ferrisServo.getPosition() + 0.005);
                //clawServo.setPosition(clawServo.getPosition() + 0.005);
            }

            if(gamepad2.b) {
                ferrisServo.setPosition(ferrisServo.getPosition() - 0.005);
                //clawServo.setPosition(clawServo.getPosition() - 0.005);
            }

            if(gamepad1.dpad_up) {
                foundationServoR.setPosition(foundationServoR.getPosition() + 0.005);
                foundationServoL.setPosition(foundationServoL.getPosition() + 0.005);
            }
            else if(gamepad1.dpad_down) {
                foundationServoR.setPosition(foundationServoR.getPosition() - 0.005);
                foundationServoL.setPosition(foundationServoL.getPosition() - 0.005);
            }

            if (gamepad1.right_bumper) {
                if (intake == 0) {
                    leftRoller.setPower(intakeSpeed);
                    rightRoller.setPower(intakeSpeed);
                    intake = 1;
                } else {
                    leftRoller.setPower(0);
                    rightRoller.setPower(0);
                    intake = 0;
                }
            }

            if (gamepad1.left_bumper) {
                if (intake == 0) {
                    leftRoller.setPower(-intakeSpeed);
                    rightRoller.setPower(-intakeSpeed);
                    intake = -1;
                } else {
                    leftRoller.setPower(0);
                    rightRoller.setPower(0);
                    intake = 0;
                }
            }

            telemetry.addData("Controls", "x");

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

            if(gamepad1.y) {
                driveMode = DriveMode.CARTESIAN;
            }

            driverControl();

            mtrFL.setPower(powFL);
            mtrFR.setPower(powFR);
            mtrBL.setPower(powBL);
            mtrBR.setPower(powBR);

//            if (gamepad2.right_stick_y > 0.1 && mtrVertical.getCurrentPosition() < VERTICAL_MAX) {
//                mtrVertical.setPower(gamepad2.right_stick_y / 3);
//            } else if (gamepad2.right_stick_y < -0.1 && mtrVertical.getCurrentPosition() > VERTICAL_MIN) {
//                mtrVertical.setPower(gamepad2.right_stick_y / 3);
//            } else if (gamepad2.right_stick_y < 0.1 && gamepad2.right_stick_y > -0.1) {
//                mtrVertical.setPower(0);
//            } else {
//                mtrVertical.setPower(0);
//            }

            if (gamepad2.right_stick_y > 0.1) {
                mtrVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mtrVertical.setPower(gamepad2.right_stick_y / 2);
            } else if (gamepad2.right_stick_y < -0.1) {
                mtrVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mtrVertical.setPower(gamepad2.right_stick_y);
            } else {
                mtrVertical.setPower(0);
            }

            //3 fl
            //2 fr
            //1 bl
            //0 br

            telemetry.addData("Left Grab: ", foundationServoL.getPosition());
            telemetry.addData("Right Grab: ", foundationServoR.getPosition());
            telemetry.addData("Rotate Servo: ", rotationServo.getPosition());
            telemetry.addData("Ferris Servo: ", ferrisServo.getPosition());
            telemetry.addData("Claw Servo: ", clawServo.getPosition());
            telemetry.addData("Right Intake:", rightRoller.getPower());
            telemetry.addData("Left Intake", leftRoller.getPower());
            telemetry.addData("Intake", intake);
//            telemetry.addData("Mtr powers", " " + powFL + powFR + powBL + powBR +
//                    mtrVertical.getPower() + " ");
//            telemetry.addData("Front Roller Forward", sFrontIntake.getPosition());
//            telemetry.addData("Front Roller Forward", frontRollerDirection);
//            telemetry.addData("Middle Roller Forward", middleRollerDirection);
//            telemetry.addData("vertical lift", mtrVertical.getCurrentPosition());
//            telemetry.addData("imu angle", getHeading());
//            telemetry.addData("drive mode", driveMode);
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
            // if the drive mode is Cartesian, we run the standard mecanum drive drive system
            powFL = g1[1] + g1[2] + g1[0];
            powFR = g1[1] - g1[2] - g1[0];
            powBL = g1[1] + g1[2] - g1[0];
            powBR = g1[1] - g1[2] + g1[0];
        }
    }

    public void initServos() {
        // in order of configuration
        rotationServo = hardwareMap.get(Servo.class, "rotation_servo");
        ferrisServo = hardwareMap.get(Servo.class, "ferris_servo");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        foundationServoL = hardwareMap.get(Servo.class, "foundation_left");
        foundationServoR = hardwareMap.get(Servo.class, "foundation_right");

        rotationServo.setPosition(0.585);
        ferrisServo.setPosition(0.5594444444);
        clawServo.setPosition(0.50);
        foundationServoL.setPosition(0.205);
        foundationServoR.setPosition(0.14444);

        rotationServo.setDirection(Servo.Direction.FORWARD);
        ferrisServo.setDirection(Servo.Direction.FORWARD);
        foundationServoR.setDirection(Servo.Direction.FORWARD);
        foundationServoL.setDirection(Servo.Direction.REVERSE);
        clawServo.setDirection(Servo.Direction.FORWARD);
    }

    public void initMotors() {
        leftRoller = hardwareMap.get(DcMotorEx.class, "leftRoller");
        rightRoller = hardwareMap.get(DcMotorEx.class, "rightRoller");

        leftRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRoller.setDirection(DcMotorSimple.Direction.REVERSE);

        ///////////////////////////////////////////////////////////////////////////////////////////

        mtrFL = hardwareMap.get(DcMotor.class, "fl");
        mtrFR = hardwareMap.get(DcMotor.class, "fr");
        mtrBL = hardwareMap.get(DcMotor.class, "bl");
        mtrBR = hardwareMap.get(DcMotor.class, "br");
        mtrVertical = hardwareMap.get(DcMotorEx.class, "vertical");

        mtrFL.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrFR.setDirection(DcMotorSimple.Direction.FORWARD);
        //backleft and backright are switched
        mtrBL.setDirection(DcMotorSimple.Direction.FORWARD);
        mtrBR.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrVertical.setDirection(DcMotorSimple.Direction.REVERSE);

        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mtrFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // encoder is reset to 0 at whatever starting position it is in
        mtrVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 0 br
        // 1 bl
        // 2 fr
        // 3 fl
    }

    // all the way out parallel to the ground is 0.55
    // middle position is 0.23
    // all the one down is 0.20

    public void extakeSetup() {
        mtrVertical.setTargetPosition(1200);
        mtrVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrVertical.setPower(0.25);

        ferrisServo.setPosition(0.577);      //ferris servo has limits 0.577 and 0.0522
        rotationServo.setPosition(0.03);  //rotation servo has limits 0.03 and 0.54

        mtrVertical.setTargetPosition(-600);
        mtrVertical.setPower(0.25);
    }

    public void extakeReturn() {
        mtrVertical.setTargetPosition(1200);
        mtrVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrVertical.setPower(0.25);

        ferrisServo.setPosition(0.595);
        rotationServo.setPosition(0.4049999999);

        mtrVertical.setTargetPosition(-600);
        mtrVertical.setPower(0.25);
    }
}
