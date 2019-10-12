package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/***
 *
 */



public class Drivetrain implements Subassembly {
    DcMotor mtrFL, mtrFR, mtrBL, mtrBR;
    LinearOpMode caller;
    HardwareMap hwMap;


    enum DrivetrainMotors {
        FL, FR, BL, BR;

        public void init() {
            
        }
    }


    public Drivetrain(LinearOpMode caller) {
        this.caller = caller;
    }


    @Override
    public void init() {
        mtrFL = caller.hardwareMap.get(DcMotor.class, ConfigurationData.DRIVETRAIN_MOTOR_NAMES[0]);
        mtrFR = caller.hardwareMap.get(DcMotor.class, ConfigurationData.DRIVETRAIN_MOTOR_NAMES[1]);
        mtrBL = caller.hardwareMap.get(DcMotor.class, ConfigurationData.DRIVETRAIN_MOTOR_NAMES[1]);
        mtrBR = caller.hardwareMap.get(DcMotor.class, ConfigurationData.DRIVETRAIN_MOTOR_NAMES[1]);

    }

    @Override
    public void status() {

    }

    //@Override
    public void run() {

    }
}
