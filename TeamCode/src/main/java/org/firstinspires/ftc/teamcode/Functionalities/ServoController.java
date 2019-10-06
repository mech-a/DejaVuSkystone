package org.firstinspires.ftc.teamcode.Functionalities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="ServoController G1A", group="Functionality Testing")
//@Disabled
public class ServoController extends LinearOpMode {

    Servo a,b,c,d;

    @Override
    public void runOpMode() {
        a = hardwareMap.get(Servo.class, "a");
        b = hardwareMap.get(Servo.class, "b");
        c = hardwareMap.get(Servo.class, "c");
        d = hardwareMap.get(Servo.class, "d");

        a.setPosition(0);
        b.setPosition(0);
        c.setPosition(0);
        d.setPosition(0);


        telemetry.addData("Stat", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a) {
                a.setPosition(a.getPosition()+0.01);
                b.setPosition(b.getPosition()+0.01);
                c.setPosition(c.getPosition()+0.01);
                d.setPosition(d.getPosition()+0.01);
            }
            else if(gamepad1.b) {
                a.setPosition(a.getPosition()-0.01);
                b.setPosition(b.getPosition()-0.01);
                c.setPosition(c.getPosition()-0.01);
                d.setPosition(d.getPosition()-0.01);
            }

            telemetry.addData("A", gamepad1.a);
            telemetry.update();
            sleep(25);
        }
    }
}
