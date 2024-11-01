package org.firstinspires.ftc.teamcode.opMode.dev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;

@TeleOp
public class FoozPadDev extends OpMode {
    FoozPad foozPad1;
    boolean y = false;

    @Override
    public void init() {
        foozPad1 = new FoozPad(gamepad1);
    }

    @Override
    public void loop() {
        if (GamepadStatic.wasJustPressed(foozPad1, GamepadStatic.Input.A)) y = true;
        foozPad1.update();
        telemetry.addData("Current: ", foozPad1.gamepad.a);
        telemetry.addData("Previous: ", foozPad1.previous.a);
        telemetry.addData("y: ", true);
    }
}
