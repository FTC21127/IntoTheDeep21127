package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic.Input;

public class Controls {
    public static Input LOW_BASKET = Input.X;
    public static Input HIGH_BASKET = Input.B;
    public static Input LOW_SPECIMEN = Input.A;
    public static Input HIGH_SPECIMEN = Input.Y;
    public static Input PRIME_INTAKE = Input.LEFT_STICK_BUTTON;
    public static Input GRAB = Input.RIGHT_STICK_BUTTON;
    public static Input RELEASE = Input.RIGHT_BUMPER;
    public static Input LOCK_SPECIMEN = Input.LEFT_BUMPER;
    public static Input ClIMB_SET = Input.RIGHT_BUMPER;
    public static Input CLIMB = Input.LEFT_BUMPER;
    public static Input DOWN_A_BIT = Input.DPAD_DOWN;
    public static Input UP_A_BIT = Input.DPAD_DOWN;
    public static Input EJECT = Input.START;
    public static Input[] SLIDES = {LOW_BASKET, HIGH_BASKET, LOW_SPECIMEN, HIGH_SPECIMEN};
    public static Input RESET_HEADING1 = Input.START;
    public static Input RESET_HEADING2 = Input.BACK;
}
