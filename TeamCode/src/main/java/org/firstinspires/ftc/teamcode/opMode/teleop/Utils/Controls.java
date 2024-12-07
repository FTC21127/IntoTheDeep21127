package org.firstinspires.ftc.teamcode.opMode.teleop.Utils;

import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic.Input;

public class Controls {
    public static Input LOW_BASKET = Input.X;
    public static Input HIGH_BASKET = Input.B;
    public static Input LOW_SPECIMEN = Input.A;
    public static Input HIGH_SPECIMEN = Input.Y;
    public static Input PRIME_INTAKE = Input.LEFT_BUMPER;
    public static Input GRAB = Input.LEFT_STICK_BUTTON;
    public static Input RELEASE = Input.RIGHT_BUMPER;
    public static Input LOCK_SPECIMEN = Input.DPAD_LEFT;
    public static Input CLIMB_SET = Input.DPAD_LEFT;
    public static Input CLIMB = Input.LEFT_BUMPER;
    public static Input EJECT = Input.DPAD_LEFT;
    public static Input[] SLIDES = {LOW_BASKET, HIGH_BASKET, LOW_SPECIMEN, HIGH_SPECIMEN};
    public static Input GRAB_SPECIMEN = Input.DPAD_UP;
    public static Input GRAB_TRANSFER = Input.DPAD_DOWN;
    public static Input RESET = Input.DPAD_RIGHT;
}