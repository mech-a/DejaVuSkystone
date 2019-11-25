package org.firstinspires.ftc.teamcode.Odometry.DejaVu;

import org.firstinspires.ftc.teamcode.Assemblies.Drivetrain;

import java.util.ArrayList;

public class GlobalPosition implements Runnable {
    private boolean threadEnabled = true;
    private Drivetrain d;
    private long iterationTime;

    //{y, x, theta}
    private ArrayList<Double> position = new ArrayList<>();

    public GlobalPosition(Drivetrain d, long iterationTime) {
        this.d = d;
        this.iterationTime = iterationTime;
        for(Double aDouble : d.getCurrentEncoderValues())
            position.add(aDouble);
    }


    @Override
    public void run() {
        while(threadEnabled) {
            setPositionArrays();
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

    }

    public ArrayList getPositionArray() {
        return position;
    }

    public void shutdown() {
        threadEnabled = false;
    }
}
