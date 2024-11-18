package org.firstinspires.ftc.teamcode.opMode.dev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;

@TeleOp
public class FoozPadDev extends OpMode {
    FoozPad foozPad1;
    boolean y = false;

    @Override
    public void init() {
        foozPad1 = new FoozPad(gamepad1);
        gamepad1.setLedColor((double) 159 /255, (double) 135 /255, (double) 211 /255, Gamepad.LED_DURATION_CONTINUOUS);
    }

    @Override
    public void loop() {
        foozPad1.update();
        telemetry.addData("Current: ", foozPad1.gamepad.a);
        telemetry.addData("Previous: ", foozPad1.previous.a);
        if (foozPad1.gamepad.a && foozPad1.previous.a) telemetry.addData("IT ","WORKS (KINDA)");
    }
}
