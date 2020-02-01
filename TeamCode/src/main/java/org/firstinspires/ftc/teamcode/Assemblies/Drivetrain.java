package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;

public class Drivetrain {
    // Motor index is as follows:
    //   F
    //  ___
    // |0 1|
    // |3 2|
    //  ¯¯¯
    //   B

    private static final int NUM_MOTORS_DT = 4;
    private boolean isInitialized = false;

    DcMotor mtrFL, mtrFR, mtrBR, mtrBL;
    ArrayList<DcMotor> motors;

    LinearOpMode caller;
    HardwareMap hwMap;
    Telemetry telemetry;

    public enum MotorOrientation {
      // 0, 1, 2, 3
        FL,FR,BR,BL
    }

    public Drivetrain(LinearOpMode aCaller) {
        this.caller = aCaller;
        hwMap = caller.hardwareMap;
        telemetry = caller.telemetry;
    }

    public void init(HardwareMap aHwMap) {
        hwMap = aHwMap;
        if(hwMap == null)
            telemetry.addData("Stat", "Null HWMAP");
        telemetry.update();

        motors = new ArrayList<>();
        //Depends on order of DRIVETRAIN_MOTOR_NAMES. Do not tamper with the order
        for(int i = 0; i<NUM_MOTORS_DT; i++) {
            DcMotor temp;
            temp = hwMap.get(DcMotor.class, ConfigurationData.DRIVETRAIN_MOTOR_NAMES[i]);

            if(i%3==0)
                //we have a motor on the left side of the robot
                temp.setDirection(DcMotor.Direction.REVERSE);
            else
                //motor on right side
                temp.setDirection(DcMotor.Direction.FORWARD);

            temp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            temp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motors.add(temp);
        }
        isInitialized = true;
    }

    public void status() {
        telemetry.addData("Drivetrain Assem Init?", isInitialized);
    }

    //TODO shift power normalization into this function, bad coding practice rn
    public void setPowers(double fl, double fr, double br, double bl) {
//        mtrFL.setPower(fl);
//        mtrFR.setPower(fr);
//        mtrBR.setPower(br);
//        mtrBL.setPower(bl);

        motors.get(0).setPower(fl);
        motors.get(0).setPower(fr);
        motors.get(0).setPower(br);
        motors.get(0).setPower(bl);

    }

    public void setPowers(double[] powers) {
        for(int i = 0; i<powers.length; i++)
            motors.get(i).setPower(i);
    }
}
