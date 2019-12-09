package org.firstinspires.ftc.teamcode.Odometry.DejaVu;

import org.firstinspires.ftc.teamcode.Assemblies.Drivetrain;

import java.util.ArrayList;

public class GlobalPosition implements Runnable {
    //Thread should continue running, if false, nothing will run
    private boolean threadEnabled = true;
    //To access the odometry wheels/encoders, should be passed in by caller
    private Drivetrain d;
    //refresh rate
    private long iterationTime;

    //{y, x, theta} is what this will have at the end, currently just (y,x)
    private ArrayList<Double> position = new ArrayList<>();

    public GlobalPosition(Drivetrain d, long iterationTime) {
        this.d = d;
        this.iterationTime = iterationTime;
        //gets away from any default values
        for(Double aDouble : d.getCurrentEncoderValues())
            position.add(aDouble);
    }

    //error = goal - actual
    //scalar = kP * error + kI * (0, current) error dt + kD * de/dt
    //integral += timePerIteration * currentEncoder
    @Override
    public void run() {
        while(threadEnabled) {
            setPositionArrays();
            //sleep try-catch in case any exceptions are thrown
            try {
                Thread.sleep(iterationTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    //assuming that we will have one encoder for the x and one for the y
    private void setPositionArrays() {
//        double newY = d.getCurrentEncoderValues().get(0);
//        double newX = d.getCurrentEncoderValues().get(1);
//
//        double dY = newY - position.get(0);
//        double dX = newX - position.get(1);

        for(int i = 0; i<d.getCurrentEncoderValues().size(); i++)
            position.set(i, d.getCurrentEncoderValues().get(i));

        //we could calculate the angle we got to now, but its not necessary because the imu
        //is p. accurate. however, this is the way to do it
        //TODO add angle calculations through x/y
    }

//    public void driveTo(double y, double x, double maxError) {
//        //inputs: error function, PID coeffs: outputs: motor powers
//        while(threadEnabled) {
//            setPositionArrays();
//            double errorX = x-position.get(1);
//
//            double pX = kP * errorX;
//            double dX = kD * (errorX-lastErrorX)/(iterationTime/1000.0);
//            double iX +=kI * (errorX*iterationTime/1000);
//            double outputScalar = pX + dX + iX;
//        }
//    }

    public ArrayList getPositionArray() {
        return position;
    }

    public void shutdown() {
        threadEnabled = false;
    }
}
