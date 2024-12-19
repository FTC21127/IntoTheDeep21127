package org.firstinspires.ftc.teamcode.opMode.teleop;

import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic.Input;

public class ControlsM3 {
    // extend intake is left stick y
    // Control diffy is right stick x
    public static Input RELEASE = Input.RIGHT_BUMPER;
    public static Input GRAB = Input.LEFT_BUMPER;
    public static Input EXTEND = Input.DPAD_LEFT;
    public static Input SPECIMEN_EJECT = Input.DPAD_UP;
    public static Input SEARCH = Input.DPAD_DOWN;
    // base / climb controls
    public static Input SPIN_CLOCKWISE = Input.X;
    public static Input SPIN_COUNTER = Input.A;
    public static Input CLIMB_SET = Input.DPAD_LEFT;
    public static Input CLIMB = Input.LEFT_BUMPER;
}