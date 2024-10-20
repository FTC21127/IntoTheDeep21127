package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;

@Config
public class Intake extends Mechanism {

    ServoEx horizontalExtendenator, v4b, claw;
    NormalizedColorSensor color;

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
        color.setGain(35);
    }

    @Override
    public void loop(Gamepad gamepad) {
        
    }

    public COLOR sampleColor(){
        if (color.getNormalizedColors().green > 0.5 && color.getNormalizedColors().red > .4) return COLOR.YELLOW;
        if (color.getNormalizedColors().red > 0.4) return COLOR.RED;
        if (color.getNormalizedColors().blue > 0.4) return COLOR.BLUE;
        return COLOR.NONE;
    }

}
