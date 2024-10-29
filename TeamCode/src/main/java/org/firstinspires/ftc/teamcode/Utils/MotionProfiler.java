package org.firstinspires.ftc.teamcode.Utils;


// borrowed code, not written by me
public class MotionProfiler {
    double maxAccel;
    double maxVel;
    double startPos;
    double targetPos;

    double accelTime = 0;
    double accelDist;
    double cruiseDist;
    double cruiseTime;
    double decelDist;
    double decelTime;
    public double traveledDist;
    double distance;
    double halfTime;
    double startVel;
    double offsetDist;
    double sign;
    double decelThreshold;
    double tempMaxVel;


    public MotionProfiler(double maxAccel, double maxVel, double offset) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
        this.offsetDist = offset;
    }

    //when you set target position/distance
    public void setTargetPos(double targetPos, double currentPos, double currentVel) {
        this.targetPos = targetPos;
        distance = targetPos - currentPos;
        sign = Math.signum(distance);
        startPos = currentPos;
        startVel = currentVel;
        accelTime = Math.abs((sign * maxVel - currentVel) / maxAccel);

        decelTime = maxVel / maxAccel;
        decelDist = 0.5 * maxAccel * decelTime * decelTime; //positive
        accelDist = Math.abs(0.5 * sign * maxAccel * accelTime * accelTime); //probably broken maybe

        if (accelDist > Math.abs(distance / 2)) {

            halfTime = Math.sqrt(Math.abs(distance / 2) / (0.5 * maxAccel)) - startVel / maxAccel;
            tempMaxVel = halfTime * maxAccel * sign + startVel * halfTime;

            accelTime = halfTime;
            decelTime = Math.abs(tempMaxVel / maxAccel);
            accelDist = 0.5 * maxAccel * accelTime * accelTime + startVel * accelTime; //
            decelDist = maxAccel * decelTime;
        }
        if (Math.abs(decelDist) >= Math.abs(distance) - offsetDist) {
            accelTime = 0;
            accelDist = 0;
        }

        cruiseDist = Math.abs(distance) - decelDist;

        decelThreshold = accelDist + cruiseDist + decelDist;
    }

    //every loop to get target velocity
    public double getTargetVel(double currentPos) {
        //traveledDist = Math.abs(currentPos-startPos) + offsetDist; //offsetDist is to prevent power from permanently being 0
        System.out.println(accelDist);


        if (Math.abs(traveledDist) < accelDist + offsetDist || (sign == -1 && traveledDist > accelDist * sign) || (sign == 1 && traveledDist < accelDist)) {

            return maxAccel * (-startVel + Math.sqrt(startVel * startVel + 4 * maxAccel * sign * currentPos)) / (2 * maxAccel * sign) * sign + startVel;

        } else if (Math.abs(traveledDist) < cruiseDist + offsetDist) {
            return maxVel * sign;
        } else if (Math.abs(traveledDist) < decelThreshold + offsetDist) {
            return maxAccel * (-startVel + Math.sqrt(startVel * startVel + 4 * maxAccel * sign * (distance - traveledDist))) / (2 * maxAccel * sign) * sign;
        } else {
            return 0;
        }
    }
}
