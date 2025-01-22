package org.firstinspires.ftc.teamcode.opMode.dev;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.opMode.teleop.Utils.FoozPadUtils.*;

@TeleOp(name = "Intake Dev", group = "Dev")
public class IntakeDev extends OpMode {
    Intake intake = new Intake(this);
    FoozPad foozPad;

    @Override
    public void init() {
        intake.init(hardwareMap);
        foozPad = new FoozPad(gamepad1);
        foozPad.runLedEffect(FoozPadColors.NURAZ_DEFAULT.colorPattern);
    }

    @Override
    public void loop() {
        foozPad.update();
        intake.loop(foozPad);
    }

    @Override
    public void stop() {
        intake.turnOffExtendy();
    }
}
