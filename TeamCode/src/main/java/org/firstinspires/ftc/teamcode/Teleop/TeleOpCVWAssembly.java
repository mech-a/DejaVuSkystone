package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Sensors;

@TeleOp(name="CV W Assembly tele", group="fx")
//@Disabled
public class TeleOpCVWAssembly extends LinearOpMode {
    Sensors s = new Sensors(this);

    @Override
    public void runOpMode() {
        s.init(hardwareMap);
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
                Sensors.SkyStoneLocation x = s.findSkystoneRed();
            sleep(1000);
        }
        s.shutdown();
    }
}
