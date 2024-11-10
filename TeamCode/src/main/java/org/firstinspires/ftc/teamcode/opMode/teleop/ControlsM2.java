package org.firstinspires.ftc.teamcode.opMode.teleop;

import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic.Input;

public class ControlsM2 {
    public static Input PRIME_INTAKE = Input.LEFT_BUMPER;
    public static Input LOW_BASKET = Input.B;
    public static Input HIGH_BASKET = Input.X;
    public static Input LOW_SPECIMEN = Input.DPAD_RIGHT;
    public static Input HIGH_SPECIMEN = Input.DPAD_LEFT;
    public static Input GRAB_INTAKE = Input.LEFT_STICK_BUTTON;
    // extend intake is right stick
    public static Input RELEASE = Input.RIGHT_BUMPER;
    public static Input[] SLIDES = {LOW_BASKET, HIGH_BASKET, LOW_SPECIMEN, HIGH_SPECIMEN};
    public static Input CLIMB_SET = Input.DPAD_LEFT;
    public static Input CLIMB = Input.LEFT_BUMPER;
}
