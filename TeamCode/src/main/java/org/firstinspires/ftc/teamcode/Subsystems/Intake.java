package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;

@Config
public class Intake extends Mechanism {

    ServoEx horizontalExtendenator, v4b, claw;
    SensorColor color;

    public enum COLOR{
        BLUE(1),
        RED(2),
        YELLOW(3),
        NONE(0);
        int i;
        COLOR(int j) {
            i = j;
        }
    }

    public Intake(OpMode opMode1) {
        this.opMode = opMode1;
    }

    @Override
    public void init(HardwareMap hwMap) {
        color = new SensorColor(hwMap, "color");
        claw = new SimpleServo(hwMap,"outtakeClaw", 0,40);
    }

    @Override
    public void loop(Gamepad gamepad) {
        
    }

    public COLOR sampleColor(){
        if (color.red() > (color.blue() + color.green()) * .75){
            return COLOR.RED;
        } else if (color.blue() > (color.red() + color.green()) * .75){
            return COLOR.BLUE;
        } else if (color.green() > 1000 && color.red() > 1000 && !(color.blue() > 1000)){
            return COLOR.YELLOW;
        } else {
            return COLOR.NONE;
        }
    }

}
