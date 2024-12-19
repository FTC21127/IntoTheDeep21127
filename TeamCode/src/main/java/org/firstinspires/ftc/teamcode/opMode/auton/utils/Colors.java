package org.firstinspires.ftc.teamcode.opMode.auton.utils;

public enum Colors{
    YELLOW(0.5,0.5,0),
    BLUE(0,0,0.5),
    RED(0.5,0,0),
    NONE(0,0,0);

    double nR, nG, nB;
    static final double tolerance = 0.075;

    Colors(double nR, double nG, double nB) {
        this.nR = nR;
        this.nG = nG;
        this.nB = nB;
    }

    public static Colors isColor(double r, double g, double b){
        Colors[] colors = {YELLOW, RED, BLUE};
        for (Colors color: colors) {
            if (Math.abs(color.nR-r) < tolerance && Math.abs(color.nG-g) < tolerance && Math.abs(color.nB-b) < tolerance){
                return color;
            }
        }
        return NONE;
    }
}