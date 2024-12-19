package org.firstinspires.ftc.teamcode.opMode.dev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.opMode.teleop.Utils.FoozPadUtils.*;

@TeleOp(name = "Drive2 Dev", group = "Dev")
public class Drive2Dev extends OpMode {
    Drivetrain drive = new Drivetrain(this);
    FoozPad foozPad;
    @Override
    public void init() {
        drive.init(hardwareMap);
        foozPad = new FoozPad(gamepad1);
        foozPad.runLedEffect(FoozPadColors.NURAZ_DEFAULT.colorPattern);
    }

    @Override
    public void loop() {
        foozPad.update();
        drive.loop(foozPad);
    }
}
