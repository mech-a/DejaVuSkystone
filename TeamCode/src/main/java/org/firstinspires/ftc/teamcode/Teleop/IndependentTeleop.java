package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Independent Teleop")
public class IndependentTeleop extends LinearOpMode{
    public DcMotor mtrFL;
    public DcMotor mtrFR;
    public DcMotor mtrBL;
    public DcMotor mtrBR;
    public DcMotor horizontalSlide;
    public DcMotor verticalSlide;

    public Servo servoTray;
    public Servo CR1, CR2, CR3, CR4;


    public void runOpMode(){

        mtrFL = hardwareMap.dcMotor.get("motor_fl");
        mtrFL.setDirection(DcMotor.Direction.FORWARD);
        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mtrFR = hardwareMap.dcMotor.get("motor_fr");
        mtrFR.setDirection(DcMotor.Direction.REVERSE);
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mtrBR = hardwareMap.dcMotor.get("motor_br");
        mtrBR.setDirection(DcMotor.Direction.REVERSE);
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mtrBL = hardwareMap.dcMotor.get("motor_bl");
        mtrBL.setDirection(DcMotor.Direction.FORWARD);
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            mtrFL.setPower(v1);
            mtrFR.setPower(v2);
            mtrBL.setPower(v3)
            mtrBR.setPower(v4);

            sleep(50);

            horizontalSlide.setPower(gamepad2.left_stick_x);
            verticalSlide.setPower(gamepad2.right_stick_y);

        }
    }
}
