package org.firstinspires.ftc.teamcode.opMode.dev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSlides;

@TeleOp(group = "Dev")
public class SlideDev extends OpMode {
    OuttakeSlides slides;

    @Override
    public void init() {
        slides = new OuttakeSlides(this);
        slides.init(hardwareMap);
        slides.restPos();
    }

    @Override
    public void loop() {
        slides.loop(gamepad1);
    }
}
