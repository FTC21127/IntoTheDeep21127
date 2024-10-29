package org.firstinspires.ftc.teamcode.fissionlib.input;

import com.qualcomm.robotcore.hardware.Gamepad;

public class FoozPad extends Gamepad {

    public Gamepad previous;
    public Gamepad gamepad;

    public FoozPad(Gamepad gamepad) {
        this.gamepad = gamepad;
        previous = gamepad;
    }

    public Gamepad getGamepad() {
        return gamepad;
    }

    public void update() {
        copy(previous, gamepad);
    }

    public void copy(Gamepad gp, Gamepad gamepad) {
        gp.a = gamepad.a;
        gp.b = gamepad.b;
        gp.x = gamepad.x;
        gp.y = gamepad.y;
        gp.start = gamepad.start;
        gp.back = gamepad.back;
        gp.guide = gamepad.guide;
        gp.dpad_down = gamepad.dpad_down;
        gp.dpad_left = gamepad.dpad_left;
        gp.dpad_right = gamepad.dpad_right;
        gp.dpad_up = gamepad.dpad_up;
        gp.left_bumper = gamepad.left_bumper;
        gp.right_bumper = gamepad.right_bumper;
        gp.left_stick_button = gamepad.left_stick_button;
        gp.right_stick_button = gamepad.right_stick_button;
        gp.left_stick_x = gamepad.left_stick_x;
        gp.left_stick_y = gamepad.left_stick_y;
        gp.right_stick_x = gamepad.right_stick_x;
        gp.right_stick_y = gamepad.right_stick_y;
        gp.left_trigger = gamepad.left_trigger;
        gp.right_trigger = gamepad.right_trigger;
    }
}