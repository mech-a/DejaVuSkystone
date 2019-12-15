package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


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

        waitForStart();

        while (opModeIsActive()) {
            currentVelocity = mtrVertical.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            /*
            if (gamepad1.a) {
                mtrFL.setPower(1);
                mtrFR.setPower(1);
                mtrBL.setPower(1);
                mtrBR.setPower(1);
            }
            */

            mtrVertical.setPower(gamepad1.right_stick_y / 2);

            if (gamepad1.a) {

            }

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
    }
}


