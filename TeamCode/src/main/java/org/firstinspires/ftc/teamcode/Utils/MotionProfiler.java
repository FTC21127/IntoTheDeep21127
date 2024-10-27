package org.firstinspires.ftc.teamcode.Utils;

public class MotionProfiler {
    double MAX_ACCEL;
    double MAX_VELO;
    double outputVelo;
    int target;
    int previousTime = 0;


    public MotionProfiler(double MAX_ACCEL, double MAX_VELO) {
        this.MAX_ACCEL = MAX_ACCEL;
        this.MAX_VELO = MAX_VELO;
        this.target = 0;
    }

    public void setTarget(int target) {
        this.target = target;
        double halfwayPos = (double) target /2;
    }

    public double trapizoidalProfile(int time, double currentVelo, int pos){
        int dt = time - previousTime;
        previousTime = time;

        double accel_dt = MAX_VELO / MAX_ACCEL;




        return MAX_VELO;
    }
}
