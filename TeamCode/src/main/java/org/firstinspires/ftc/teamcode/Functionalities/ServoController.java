package org.firstinspires.ftc.teamcode.Functionalities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="ServoController G1A", group="Functionality Testing")
//@Disabled
public class ServoController extends LinearOpMode {

    Servo a,b,c,d;
    CRServo ca, cb, cc, cd;

    @Override
    public void runOpMode() {
        a = hardwareMap.get(Servo.class, "a");
        b = hardwareMap.get(Servo.class, "b");
        c = hardwareMap.get(Servo.class, "c");
        d = hardwareMap.get(Servo.class, "d");

        ca = hardwareMap.get(CRServo.class, "ca");
        cb = hardwareMap.get(CRServo.class, "cb");
        cc = hardwareMap.get(CRServo.class, "cc");
        cd = hardwareMap.get(CRServo.class, "cd");

//        a.setPosition(1);
//        b.setPosition(0);
//        c.setPosition(1);
//        d.setPosition(0);


        telemetry.addData("Stat", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a) {
                a.setPosition(a.getPosition()+0.01);
                b.setPosition(b.getPosition()+0.01);
            }
            else if(gamepad1.b) {
                a.setPosition(a.getPosition()-0.01);
                b.setPosition(b.getPosition()-0.01);
            }
            
            if(gamepad1.x) {
                c.setPosition(c.getPosition()+0.01);
                d.setPosition(d.getPosition()+0.01);
            }
            else if(gamepad1.y) {
                c.setPosition(c.getPosition()-0.01);
                d.setPosition(d.getPosition()-0.01);
            }
            
            if(gamepad1.dpad_up) {
                ca.setPower(1);
                cb.setPower(1);
            }
            else if(gamepad1.dpad_down) {
                ca.setPower(-1);
                cb.setPower(-1);
            }

            if(gamepad1.dpad_left) {
                cc.setPower(1);
                cd.setPower(1);
            }
            else if(gamepad1.dpad_right) {
                cc.setPower(-1);
                cd.setPower(-1);
            }

            sleep(25);
        }
    }
}
