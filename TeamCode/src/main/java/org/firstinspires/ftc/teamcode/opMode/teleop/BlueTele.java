package org.firstinspires.ftc.teamcode.opMode.teleop;

import static org.firstinspires.ftc.teamcode.opMode.teleop.Utils.FoozPadUtils.FoozPadColors.NURAZ_DEFAULT;
import static org.firstinspires.ftc.teamcode.opMode.teleop.Utils.FoozPadUtils.FoozPadColors.SARAH_INTAKE;
import static org.firstinspires.ftc.teamcode.opMode.teleop.Utils.FoozPadUtils.FoozPadColors.SARAH_OUTTAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.opMode.auton.utils.Colors;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Scoring;

@TeleOp(group = "!Comp")
public class BlueTele extends OpMode {

    Robot bot = new Robot(this, Colors.BLUE);
    FoozPad gp1, gp2;
    Scoring.State previousState = Scoring.State.SCORING;

    @Override
    public void init() {
        bot.init(hardwareMap);
        gp1 = new FoozPad(gamepad1);
        gp2 = new FoozPad(gamepad2);
        gp2.runLedEffect(SARAH_INTAKE.colorPattern);
        gp1.runLedEffect(NURAZ_DEFAULT.colorPattern);
    }

    @Override
    public void loop() {
        bot.loop(gp1, gp2);
        gp1.update();
        gp2.update();
        if (!bot.getState().equals(previousState)) {
            if (bot.getState().equals(Scoring.State.INTAKE)) {
                gp2.runLedEffect(SARAH_INTAKE.colorPattern);
            } else if (bot.getState().equals(Scoring.State.SCORING)) {
                gp2.runLedEffect(SARAH_OUTTAKE.colorPattern);
            }
        }
        previousState = bot.getState();
    }
}
