package org.firstinspires.ftc.teamcode.FoozFunctions;

public class SimpleMotionProfile {

    private double power = 0;
    private int target;
    private double MAX_ACCEL;
    private double MAX_VELO;
    private int tolerance;

    public SimpleMotionProfile(double MAX_ACCEL, double MAX_VELO, int tolerance, int target) {
        this.MAX_ACCEL = MAX_ACCEL;
        this.MAX_VELO = MAX_VELO;
        this.tolerance = tolerance;
        this.target = target;
    }

    public SimpleMotionProfile(int tolerance, double MAX_VELO, double MAX_ACCEL) {
        this(MAX_ACCEL, MAX_VELO, tolerance, 0);
    }

    public void setTarget(int target) {
        this.target = target;
    }

    public void setMAX_VELO(double MAX_VELO) {
        this.MAX_VELO = MAX_VELO;
    }

    public void setMAX_ACCEL(double MAX_ACCEL) {
        this.MAX_ACCEL = MAX_ACCEL;
    }

    public void setTolerance(int tolerance){
        this.tolerance = tolerance;
    }

    public double calculate(int currentPos, int targetPos){
        target = targetPos;
        double error = target - currentPos;
        if (Math.abs(error)>tolerance) {
            power = (Math.abs(currentPos) < Math.abs(target / 2) ? power + MAX_ACCEL : power - MAX_ACCEL);
        } else {
            power = 0;
        }

        return (power>MAX_VELO)?MAX_VELO: Math.max(power, -MAX_VELO);
    }

    public double calculate(int currentPos){
       return calculate(currentPos, target);
    }
}
