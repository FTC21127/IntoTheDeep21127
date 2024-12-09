package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.Deposit.RELEASE;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;


@Config
public class Intake extends Mechanism {

    Servo horizontalExtendenator;
    Servo v4b;
    Servo claw;

    COLOR alliance;

    public enum COLOR{
        BLUE,
        RED,
        YELLOW,
        NONE
    }

    public Intake(OpMode opMode1){
        this(opMode1,COLOR.RED);
    }

    public Intake(OpMode opMode1, COLOR alliance) {
        this.opMode = opMode1;
        this.alliance = alliance;
    }

    @Override
    public void init(HardwareMap hwMap) {

    }

    @Override
    public void loop(FoozPad gamepad) {

    }
}
