package org.firstinspires.ftc.teamcode.FoozFunctions;

public class SimpleMotionProfile {

    private double power = 0; // power value
    private int target; // position you want to move to (in ticks)
    private double MAX_ACCEL; // how much the controller speeds up per loop (~50hz)
    private double MAX_VELO; // max speed (0-1)
    private int tolerance; // margin of error (in ticks)

    // input values (can be changed later)
    public SimpleMotionProfile(double MAX_ACCEL, double MAX_VELO, int tolerance, int target) {
        this.MAX_ACCEL = MAX_ACCEL;
        this.MAX_VELO = MAX_VELO;
        this.tolerance = tolerance;
        this.target = target;
    }

    // input values without target
    public SimpleMotionProfile(double MAX_ACCEL, double MAX_VELO, int tolerance) {
        this(MAX_ACCEL, MAX_VELO, tolerance, 0);
    }

    // setters
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

    // input current position (sensor data) and your target position
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
    // input current position (sensor data)
    public double calculate(int currentPos){
       return calculate(currentPos, target);
    }
}
