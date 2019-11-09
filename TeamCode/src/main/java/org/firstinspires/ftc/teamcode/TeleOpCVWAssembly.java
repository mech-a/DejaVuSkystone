package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Sensors;

@TeleOp(name="CV W Assembly tele", group="fx")
//@Disabled
public class TeleOpCVWAssembly extends LinearOpMode {
    Sensors s = new Sensors(this);

    @Override
    public void runOpMode() {
        s.init();
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a) {
                Sensors.SkyStoneLocation x = s.findSkystone();
            }
            sleep(200);
        }
    }
}
