package org.firstinspires.ftc.teamcode.fissionlib.input;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadStatic {

    public enum Input {
        NONE,
        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT,
        DPAD_RIGHT,
        A,
        B,
        X,
        Y,
        START,
        BACK,
        LEFT_BUMPER,
        RIGHT_BUMPER,
        LEFT_STICK_BUTTON,
        RIGHT_STICK_BUTTON,
        LEFT_TRIGGER,
        RIGHT_TRIGGER
    }

    public static boolean isButtonPressed(Gamepad gamepad, Input button) {
        switch (button) {
            case DPAD_UP:
                return gamepad.dpad_up;
            case DPAD_DOWN:
                return gamepad.dpad_down;
            case DPAD_LEFT:
                return gamepad.dpad_left;
            case DPAD_RIGHT:
                return gamepad.dpad_right;
            case A:
                return gamepad.a;
            case B:
                return gamepad.b;
            case X:
                return gamepad.x;
            case Y:
                return gamepad.y;
            case START:
                return gamepad.start;
            case BACK:
                return gamepad.back;
            case LEFT_BUMPER:
                return gamepad.left_bumper;
            case RIGHT_BUMPER:
                return gamepad.right_bumper;
            case LEFT_STICK_BUTTON:
                return gamepad.left_stick_button;
            case RIGHT_STICK_BUTTON:
                return gamepad.right_stick_button;
            case LEFT_TRIGGER:
                return gamepad.left_trigger > 0;
            case RIGHT_TRIGGER:
                return gamepad.right_trigger > 0;
            default:
                return false;
        }
    }

    public static boolean wasJustPressed(@NonNull FoozPad gp, Input input){
        return GamepadStatic.isButtonPressed(gp.gamepad, input) && !GamepadStatic.isButtonPressed(gp.previous, input);
    }

    public static boolean wasJustReleased(@NonNull FoozPad gp, Input input){
        return !GamepadStatic.isButtonPressed(gp.gamepad, input) && GamepadStatic.isButtonPressed(gp.previous, input);
    }

    public static boolean beenPressed(@NonNull FoozPad gp, Input input){
        return GamepadStatic.isButtonPressed(gp.gamepad, input) && GamepadStatic.isButtonPressed(gp.previous, input);
    }

    public static boolean stateJustChanged(@NonNull FoozPad gp, Input input){
        return GamepadStatic.isButtonPressed(gp.gamepad, input) != GamepadStatic.isButtonPressed(gp.previous, input);
    }
    public static boolean triggerPressed(@NonNull FoozPad gp, Input input){
        switch (input) {
            case LEFT_TRIGGER:
                return gp.gamepad.left_trigger>0;
            case RIGHT_TRIGGER:
                return gp.gamepad.right_trigger>0;
            default:
                return false;
        }
    }
    public static boolean triggerPressed(@NonNull FoozPad gp, Input input, double threshold){
        switch (input) {
            case LEFT_TRIGGER:
                return gp.gamepad.left_trigger>threshold;
            case RIGHT_TRIGGER:
                return gp.gamepad.right_trigger>threshold;
            default:
                return false;
        }
    }
}
