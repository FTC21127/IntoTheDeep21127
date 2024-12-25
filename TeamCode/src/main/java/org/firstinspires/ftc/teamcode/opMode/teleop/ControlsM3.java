package org.firstinspires.ftc.teamcode.opMode.teleop;

import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic.Input;

public class ControlsM3 {
    // extendy control is left stick y
    // diffy control is right stick x
    public static final Input RELEASE = Input.RIGHT_BUMPER;
    public static final Input INTAKE = Input.LEFT_BUMPER;
    public static final Input SPECIMEN_POS = Input.DPAD_UP;
    public static final Input LOW_BASKET = Input.X;
    public static final Input HIGH_BASKET = Input.B;
    public static final Input LOW_SPECIMEN = Input.A;
    public static final Input HIGH_SPECIMEN = Input.Y;
    public static final Input[] SLIDES = {LOW_BASKET, HIGH_BASKET, LOW_SPECIMEN, HIGH_SPECIMEN};
    // base / climb controls
    public static final Input SPIN_CLOCKWISE = Input.X;
    public static final Input SPIN_COUNTER = Input.B;
    public static final Input FLIP = Input.Y;
    public static final Input CLIMB_SET = Input.DPAD_LEFT;
    public static final Input CLIMB = Input.LEFT_BUMPER;
}