package org.firstinspires.ftc.teamcode.Assemblies;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;


public class DrivetrainAuton implements Subassembly {
    DcMotor mtrFL, mtrFR, mtrBL, mtrBR;
    LinearOpMode caller;
    Telemetry telemetry;

    //DT Specs:
    public static final double HD_COUNTS_PER_REV = 560;
    //in inches
    public static final double WHEEL_DIAM = 4;
    public static final double DRIVE_GEAR_RATIO = 1;
    public static final double HD_COUNTS_PER_INCH = getCountsPerInch(DRIVE_GEAR_RATIO, HD_COUNTS_PER_REV, WHEEL_DIAM);

    int driveMtrTarget = 1;

    private boolean ccwRotation = false;
    private Orientation angles;

    private double heading;

    private BNO055IMU imu;
    private BNO055IMU.Parameters gyroParameters;

    public DrivetrainAuton(LinearOpMode caller) {
        this.caller = caller;
        telemetry = caller.telemetry;
    }

    /* TODO note that we have switched the order of motors in our TeleOp and robot. Refer to @see Drivetrain */
    @Override
    public void init() {
        mtrFL = caller.hardwareMap.get(DcMotor.class, ConfigurationData.DRIVETRAIN_MOTOR_NAMES[0]);
        mtrFR = caller.hardwareMap.get(DcMotor.class, ConfigurationData.DRIVETRAIN_MOTOR_NAMES[1]);
        mtrBL = caller.hardwareMap.get(DcMotor.class, ConfigurationData.DRIVETRAIN_MOTOR_NAMES[2]);
        mtrBR = caller.hardwareMap.get(DcMotor.class, ConfigurationData.DRIVETRAIN_MOTOR_NAMES[3]);

        mtrFL.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrFR.setDirection(DcMotorSimple.Direction.FORWARD);
        mtrBL.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrBR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void status() {

    }

    //@Override
    public void run() {

    }

    public enum Direction {
        FWD,BACK,LEFT,RIGHT
    }

    private static double getCountsPerInch(double gearRatio, double countsRevolution, double diam) {
        return (gearRatio * countsRevolution / (Math.PI * diam));
    }

    public void translate(Direction dir, double inches, double speed) {
        driveMtrTarget = (int) (Math.abs(inches) * HD_COUNTS_PER_INCH);

        int a, b;
        for (int i = 0; i<4 && !caller.isStopRequested(); i++) {
            switch (dir) {
                case LEFT:
                    a = -1;
                    b = 1;
                    break;
                case RIGHT:
                    a = 1;
                    b = -1;
                    break;
                case FWD:
                    a = 1;
                    b = 1;
                    break;
                case BACK:
                    a = -1;
                    b = -1;
                    break;
                default:
                    telemetry.addData("Err", "Unknown dir %s", dir.toString());
                    telemetry.update();
                    a = 0;
                    b = 0;
                    break;
            }

            mtrFL.setTargetPosition(a * driveMtrTarget);
            mtrFR.setTargetPosition(b * driveMtrTarget);
            mtrBL.setTargetPosition(b * driveMtrTarget);
            mtrBR.setTargetPosition(a * driveMtrTarget);

            mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(!caller.isStopRequested()){
            mtrFL.setPower(speed);
            mtrFR.setPower(speed);
            mtrBL.setPower(speed);
            mtrBR.setPower(speed);
        }

        while(!caller.isStopRequested() &&
                ((mtrFL.isBusy()) && (mtrFR.isBusy()) && (mtrBL.isBusy()) && (mtrBR.isBusy())) ) {
            //TODO change telemetry name to enum
            telemetry.addData("0mtrFl", "%7d : %7d",
                    mtrFL.getCurrentPosition(), driveMtrTarget);
            telemetry.addData("1mtrFR", "%7d : %7d",
                    mtrFR.getCurrentPosition(), driveMtrTarget);
            telemetry.addData("2mtrBR", "%7d : %7d",
                    mtrBL.getCurrentPosition(), driveMtrTarget);
            telemetry.addData("3mtrBL", "%7d : %7d",
                    mtrBR.getCurrentPosition(), driveMtrTarget);

            telemetry.update();
        }

        if(!caller.isStopRequested()){
            mtrFL.setPower(0);
            mtrFR.setPower(0);
            mtrBL.setPower(0);
            mtrBR.setPower(0);
        }

        if(!caller.isStopRequested()) {
            mtrFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtrFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtrBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtrBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


    }

    public void translate(double angle, double inches, double speed) {
        angle = angle + Math.PI / 4;

        double x = inches * Math.cos(angle);
        double y = inches * Math.sin(angle);

        mtrFL.setPower(x);
        mtrFR.setPower(y);
        mtrBR.setPower(x);
        mtrBL.setPower(y);

    }

    //Directions can be abbreviated to 'cw' or 'ccw'
    //It does not currently reset the gyro sensor
    /**
     * Rotate is a method that uses he IMU within the Rev Hubs
     * and rotates within autonomous. This method takes params
     * that allow it to determine which motors go which way
     * based on clockwise(cw) and counterclockwise(ccw). It uses
     * other methods to determine when the desired angle has been
     * reached and stop as well as a method to reset the gyroscopic
     * sensor.
     * @param direction
     * @param speed
     * @param angle
     */
    public void rotate(String direction, double speed, double angle) {

        double powL, powR;

        if(direction.equals("cw") || direction.equals("clockwise")) {
            ccwRotation = false;
            powL = speed;
            powR = -speed;
        }
        else {
            ccwRotation = true;
            powL = -speed;
            powR = speed;
        }

        if (!caller.isStopRequested()) {
            mtrFL.setPower(powL);
            mtrFR.setPower(powR);
            mtrBL.setPower(powL);
            mtrBR.setPower(powR);

        }

        //telemetry.addData("Rotating:", "%7d, %7s");

        //priming
        refreshAngle();


        while(reachedAngle(angle) && !caller.isStopRequested()) {
            refreshAngle();
            //telemetry.addData("Angle", "%7d : %7d", imu.getAngularOrientation(), angle);
        }

        telemetry.addData("heading","%7f %7f", heading, angle);
        telemetry.update();

        if (caller.isStopRequested()) {
            mtrFL.setPower(0);
            mtrFR.setPower(0);
            mtrBL.setPower(0);
            mtrBR.setPower(0);
        }

    }

    private boolean reachedAngle(double angle) {
        if (ccwRotation)
            return heading < angle;
        else
            return heading > -angle;
    }

    //Currently normalizes angle as well
    public void refreshAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

        heading = AngleUnit.DEGREES.normalize(heading);

    }
    public double getHeading(){
        refreshAngle();
        return heading;
    }

}


