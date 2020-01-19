package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class MaxVelocityTest extends LinearOpMode {
    DcMotorEx mtrFL,mtrFR,mtrBL,mtrBR;;
    DcMotorEx mtrVertical;
    double currentVelocity;
    double maxVelocity = 0.0;

    @Override
    public void runOpMode() {
        mtrFL = hardwareMap.get(DcMotorEx.class, "fl");
        mtrFR = hardwareMap.get(DcMotorEx.class, "fr");
        mtrBL = hardwareMap.get(DcMotorEx.class, "bl");
        mtrBR = hardwareMap.get(DcMotorEx.class, "br");
        mtrVertical = hardwareMap.get(DcMotorEx.class, "vertical");

        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mtrFL.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrFR.setDirection(DcMotorSimple.Direction.FORWARD);
        mtrBL.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrBR.setDirection(DcMotorSimple.Direction.FORWARD);

        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            currentVelocity = mtrFL.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }


            if (gamepad1.a) {
                mtrFL.setPower(1);
                mtrFR.setPower(1);
                mtrBL.setPower(1);
                mtrBR.setPower(1);
            } else {
                mtrFL.setPower(0);
                mtrFR.setPower(0);
                mtrBL.setPower(0);
                mtrBR.setPower(0);
            }



            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
    }
}


