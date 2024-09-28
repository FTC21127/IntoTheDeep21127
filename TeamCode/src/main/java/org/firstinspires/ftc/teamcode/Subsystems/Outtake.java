package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake extends SubsystemBase {

    private final Motor slide1, slide2;
    private final Servo bucket;
    public Outtake(HardwareMap hMap, String slide1Name, String slide2Name, String servoName) {
        slide1 = new Motor(hMap, slide1Name);
        slide2 = new Motor(hMap, slide2Name);
        bucket = hMap.get(Servo.class, servoName);
    }

    public void dropSample(){
        bucket.setPosition(0);
    }
    public void restBucket(){
        bucket.setPosition(0.5);
    }
    public void setPower(double power){
        slide1.set(power);
        slide2.set(-power);
    }
}
