package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;

public class Climb extends Mechanism {

    Servo lock;

    public static double LOCK_POS = 0;
    public static double UNLOCK_POS = 0.4;

    public Climb(OpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        lock = hwMap.get(Servo.class, "lock");
    }

    public void lockServo(){
        lock.setPosition(LOCK_POS);
    }

    public void unLock(){
        lock.setPosition(UNLOCK_POS);
    }

    @Override
    public void loop(FoozPad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.A)){
            lockServo();
        }
    }
}
