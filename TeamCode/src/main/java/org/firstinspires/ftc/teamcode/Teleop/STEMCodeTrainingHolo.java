package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Pushbot: Teleop POV", group="Pushbot")
public class STEMCodeTrainingHolo extends LinearOpMode {

    public DcMotor mtr0;
    public DcMotor mtr1;
    public DcMotor mtr2;
    public DcMotor mtr3;
    public double Pow0, Pow1, Pow2, Pow3;
    public double modifier = 0.33;

    public double drive;
    public double turn;

    public double a, b, c, d;

    public void runOpMode(){

        mtr0 = hardwareMap.dcMotor.get("motor_0");
        mtr0.setDirection(DcMotor.Direction.FORWARD);
        mtr0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mtr1 = hardwareMap.dcMotor.get("motor_1");
        mtr1.setDirection(DcMotor.Direction.REVERSE);
        mtr1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mtr2 = hardwareMap.dcMotor.get("motor_2");
        mtr2.setDirection(DcMotor.Direction.REVERSE);
        mtr2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mtr3 = hardwareMap.dcMotor.get("motor_3");
        mtr3.setDirection(DcMotor.Direction.FORWARD);
        mtr3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            a = -gamepad1.left_stick_y;
            b = gamepad1.left_stick_x;
            c = gamepad1.right_stick_x;

            Pow0 = a+b+c;
            Pow1 = a-b-c;
            Pow2 = a+b-c;
            Pow3 = a-b+c;

            mtr0.setPower(Pow0*modifier);
            mtr3.setPower(Pow3*modifier);
            mtr1.setPower(Pow1*modifier);
            mtr2.setPower(Pow2*modifier);

            sleep(50);

        }
    }
}
