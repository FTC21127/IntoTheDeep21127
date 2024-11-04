package org.firstinspires.ftc.teamcode.opMode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;

@TeleOp(group = "Dev")
public class SlideDev extends OpMode {
    OuttakeSlides slides;
    FoozPad gp1;
    Telemetry telemetry1;

    @Override
    public void init() {
        slides = new OuttakeSlides(this);
        slides.init(hardwareMap);
        slides.restPos();
        gp1 = new FoozPad(gamepad1);
        telemetry1 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        gp1.update();
        slides.update();
        slides.loop(gp1);
        slides.telemetry(telemetry);
    }
}
