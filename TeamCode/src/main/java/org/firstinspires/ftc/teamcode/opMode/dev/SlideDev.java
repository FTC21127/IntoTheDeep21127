package org.firstinspires.ftc.teamcode.opMode.dev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;

@TeleOp(group = "Dev")
public class SlideDev extends OpMode {
    OuttakeSlides slides;
    FoozPad gp1;

    @Override
    public void init() {
        slides = new OuttakeSlides(this);
        slides.init(hardwareMap);
        slides.restPos();
        gp1 = new FoozPad(gamepad1);
    }

    @Override
    public void loop() {
        slides.update();
        slides.loop(gp1);
    }
}
