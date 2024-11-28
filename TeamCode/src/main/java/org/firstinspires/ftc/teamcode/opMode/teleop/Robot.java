package org.firstinspires.ftc.teamcode.opMode.teleop;

import static org.firstinspires.ftc.teamcode.opMode.teleop.Utils.FoozPadColors.SARAH_INTAKE;
import static org.firstinspires.ftc.teamcode.opMode.teleop.Utils.FoozPadColors.SARAH_OUTTAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.opMode.teleop.Utils.FoozPadColors;

@TeleOp
public class Robot extends OpMode {
    Scoring bot = new Scoring(this, Intake.COLOR.RED);
    FoozPad gp1, gp2;
    Scoring.State previousState = Scoring.State.SCORING;

    @Override
    public void init() {
        bot.init(hardwareMap);
        gp1 = new FoozPad(gamepad1);
        gp2 = new FoozPad(gamepad2);
        gp2.assignedPad.runLedEffect(SARAH_INTAKE.colorPattern);
        gp1.assignedPad.runLedEffect(FoozPadColors.NURAZ_DEFAULT.colorPattern);
    }

    @Override
    public void start() {
        bot.resetSlide();
    }

    @Override
    public void loop() {
        bot.loop(gp1, gp2);
        telemetry.addData("Trigger? ", gp2.gamepad.right_trigger +  gp2.gamepad.left_trigger);
        gp1.update();
        gp2.update();
        if (!bot.state.equals(previousState)) {
            if (bot.state.equals(Scoring.State.INTAKE)) {
                gp2.assignedPad.runLedEffect(SARAH_INTAKE.colorPattern);
            } else if (bot.state.equals(Scoring.State.SCORING)) {
                gp2.assignedPad.runLedEffect(SARAH_OUTTAKE.colorPattern);
            } else {
                gp2.assignedPad.setLedColor(1,1,1, Gamepad.LED_DURATION_CONTINUOUS);
            }
        }
        previousState = bot.getState();
    }
}
