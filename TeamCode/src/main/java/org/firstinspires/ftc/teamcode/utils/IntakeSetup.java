package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.opMode.teleop.Utils.FoozPadUtils;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "Intake Setup", group = "Testing")
public class IntakeSetup extends OpMode {
    Intake intake = new Intake(this);
    FoozPad foozPad;

    @Override
    public void init() {
        intake.init(hardwareMap);
        foozPad = new FoozPad(gamepad1);
        foozPad.runLedEffect(FoozPadUtils.FoozPadColors.NURAZ_DEFAULT.colorPattern);
    }

    @Override
    public void loop() {
        foozPad.update();
        intake.setUp(foozPad);
    }
}