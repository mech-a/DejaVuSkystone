package org.firstinspires.ftc.teamcode.Functionalities;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="MotorController", group="Functionality Testing")
//@Disabled
public class MotorController extends LinearOpMode {

    DcMotor a,b,c,d;

    @Override
    public void runOpMode() {
        a = hardwareMap.get(DcMotor.class, "a");
        b = hardwareMap.get(DcMotor.class, "b");
        c = hardwareMap.get(DcMotor.class, "c");
        d = hardwareMap.get(DcMotor.class, "d");

        a.setDirection(DcMotorSimple.Direction.FORWARD);
        b.setDirection(DcMotorSimple.Direction.FORWARD);
        c.setDirection(DcMotorSimple.Direction.FORWARD);
        d.setDirection(DcMotorSimple.Direction.FORWARD);

        d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        a.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        c.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Stat", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            a.setPower(-gamepad1.left_stick_y);
            b.setPower(-gamepad1.left_stick_y);
            c.setPower(-gamepad1.right_stick_y);
            d.setPower(-gamepad1.right_stick_y);
            telemetry.addData("Joystick Left", -gamepad1.left_stick_y);
            telemetry.addData("Joystick Right", -gamepad1.right_stick_y);
            telemetry.addData("mtr d encoder", d.getCurrentPosition());

            //vertical zero: 201
            //vertical max: 3330


            telemetry.update();
        }
    }
}
