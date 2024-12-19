package org.firstinspires.ftc.teamcode.fissionlib.input;

import com.qualcomm.robotcore.hardware.Gamepad;

public class FoozPad extends Gamepad {

    public Gamepad previous = new Gamepad();
    public Gamepad gamepad = new Gamepad();
    private Gamepad assignedPad;

    public FoozPad(Gamepad gamepad) {
        this.assignedPad = gamepad;
        this.gamepad.copy(assignedPad);
    }

    public Gamepad getGamepad() {
        return gamepad;
    }

    public void update() {
         previous.copy(gamepad);
         gamepad.copy(assignedPad);
    }

    @Override
    public void runLedEffect(LedEffect effect) {
        assignedPad.runLedEffect(effect);
    }

    @Override
    public void runRumbleEffect(RumbleEffect effect) {
        assignedPad.runRumbleEffect(effect);
    }

    public void copy(Gamepad gamepad) {
        previous.a = gamepad.a;
        previous.b = gamepad.b;
        previous.x = gamepad.x;
        previous.y = gamepad.y;
        previous.start = gamepad.start;
        previous.back = gamepad.back;
        previous.guide = gamepad.guide;
        previous.dpad_down = gamepad.dpad_down;
        previous.dpad_left = gamepad.dpad_left;
        previous.dpad_right = gamepad.dpad_right;
        previous.dpad_up = gamepad.dpad_up;
        previous.left_bumper = gamepad.left_bumper;
        previous.right_bumper = gamepad.right_bumper;
        previous.left_stick_button = gamepad.left_stick_button;
        previous.right_stick_button = gamepad.right_stick_button;
        previous.left_stick_x = gamepad.left_stick_x;
        previous.left_stick_y = gamepad.left_stick_y;
        previous.right_stick_x = gamepad.right_stick_x;
        previous.right_stick_y = gamepad.right_stick_y;
        previous.left_trigger = gamepad.left_trigger;
        previous.right_trigger = gamepad.right_trigger;
    }
}