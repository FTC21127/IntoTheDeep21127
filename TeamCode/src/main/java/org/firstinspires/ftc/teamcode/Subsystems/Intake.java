package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;

@Config
public class Intake extends Mechanism {

    ServoEx horizontalExtendenator, v4b, claw;
    NormalizedColorSensor color; // give color values in range from 0 - 1

    //Positions to be tuned
    public static double BAR_DOWN = 0;
    public static double BAR_TRANSFER = 1;
    public static double BAR_NEUTRAL = 0.8;
    public double GRIP = 0;
    public double RELEASE = 1;
    public static double SLIDE_COMPRESS = 0;
    public static double SLIDE_NEUTRAL = .5;

    public enum COLOR{
        BLUE,
        RED,
        YELLOW,
        NONE
    }

    public Intake(OpMode opMode1) {
        this.opMode = opMode1;
    }

    @Override
    public void init(HardwareMap hwMap) {
        color = hwMap.get(NormalizedColorSensor.class, "color");
        claw = new SimpleServo(hwMap,"intakeClaw", 0,40);
        v4b = new SimpleServo(hwMap,"v4b",0,160);
        horizontalExtendenator = new SimpleServo(hwMap, "intakeSlides", 0, 110);

        color.setGain(35);
    }

    public void closeClaw(){
        claw.setPosition(GRIP);
    }

    public void openClaw(){
        claw.setPosition(RELEASE);
    }

    public void setSlidePos(double pos){
        horizontalExtendenator.setPosition(pos);
    }

    @Override
    public void loop(FoozPad gamepad) {
        
    }

    public COLOR sampleColor(){
        if (color.getNormalizedColors().green > 0.5 && color.getNormalizedColors().red > .4) return COLOR.YELLOW;
        if (color.getNormalizedColors().red > 0.4) return COLOR.RED;
        if (color.getNormalizedColors().blue > 0.4) return COLOR.BLUE;
        return COLOR.NONE;
    }

}
