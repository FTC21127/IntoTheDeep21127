package org.firstinspires.ftc.teamcode.opMode.dev;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;

@TeleOp(name = "Claw Dev", group = "Dev")
public class IntakeDev extends OpMode {
    Intake intake = new Intake(this, Intake.COLOR.RED);
    FoozPad foozPad;

    @Override
    public void init() {
        intake.init(hardwareMap);
        foozPad = new FoozPad(gamepad1);
    }

    @Override
    public void loop() {
        foozPad.update();
        intake.loop(foozPad);
        telemetry.addData("Colour: ", intake.sampleColor());
    }
}
