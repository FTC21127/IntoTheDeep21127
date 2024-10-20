package org.firstinspires.ftc.teamcode.opMode.dev;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(name = "Claw Dev", group = "Dev")
public class IntakeDev extends OpMode {
    Intake intake = new Intake(this);

    @Override
    public void init() {
        intake.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Colour: ", intake.sampleColor());
    }
}
